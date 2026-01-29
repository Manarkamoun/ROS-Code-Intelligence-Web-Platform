import re
import ast
import json
import subprocess
import sys
from typing import Dict, List, Any, Set, Tuple, Optional
from dataclasses import dataclass, field
from pathlib import Path
import xml.etree.ElementTree as ET
from enum import Enum

class Severity(Enum):
    ERROR = 'error'
    WARNING = 'warning'
    INFO = 'info'

@dataclass
class ValidationIssue:
    file_path: str
    line_number: int
    issue_type: str
    severity: str  # 'error', 'warning', 'info'
    message: str
    code_snippet: str = ""
    suggested_fix: str = ""

@dataclass
class ROS2Metadata:
    """Stocke les métadonnées d'un package ROS2"""
    topics: Set[str] = field(default_factory=set)
    services: Set[str] = field(default_factory=set)
    actions: Set[str] = field(default_factory=set)
    parameters: Set[str] = field(default_factory=set)
    nodes: Set[str] = field(default_factory=set)

class ROS2Validator:
    """Syntax and logic validator for ROS2"""
    
    def __init__(self):
        self.issues: List[ValidationIssue] = []
        self.common_ros_topics = [
            '/tf', '/tf_static', '/scan', '/camera/image_raw',
            '/odom', '/imu', '/cmd_vel', '/joint_states'
        ]
        self.declared_topics: Dict[str, Set[str]] = {}
        self.declared_params: Dict[str, Set[str]] = {}
        self.used_topics: Dict[str, Set[str]] = {}
        self.used_params: Dict[str, Set[str]] = {}
        self.package_dependencies: Dict[str, Set[str]] = {}
        self.rate_objects: Dict[str, List[Tuple[int, str]]] = {}
        self.node_names: Set[str] = set()
        self.service_names: Set[str] = set()
        self.action_names: Set[str] = set()
    
    def validate_file(self, file_path: str, content: str) -> List[ValidationIssue]:
        """Validates a ROS2 file"""
        self.issues = []
        
        self.declared_topics[file_path] = set()
        self.declared_params[file_path] = set()
        self.used_topics[file_path] = set()
        self.used_params[file_path] = set()
        self.rate_objects[file_path] = []
        
        if file_path.endswith('.py'):
            self._validate_python_file(file_path, content)
        elif file_path.endswith(('.cpp', '.hpp', '.c', '.h')):
            self._validate_cpp_file(file_path, content)
        elif file_path.endswith(('.launch.py', '.launch', '.launch.xml')):
            self._validate_launch_file(file_path, content)
        elif file_path.endswith(('.yaml', '.yml')):
            self._validate_yaml_file(file_path, content)
        elif file_path.endswith(('.urdf', '.xacro')):
            self._validate_urdf_file(file_path, content)
        elif 'package.xml' in file_path:
            self._extract_package_dependencies(file_path, content)
        
        return self.issues
    
    def _is_cpp_main_file(self, file_path: str, content: str) -> bool:
        """Détermine si un fichier C++ est un point d'entrée principal"""
        file_name = Path(file_path).name.lower()
        
        # Fichiers qui sont toujours des headers (pas des mains)
        is_header = file_path.endswith(('.h', '.hpp', '.hxx', '.hh'))
        if is_header:
            return False
        
        # Fichiers nommés comme des mains
        main_file_patterns = [
            'main.cpp', 'main.cxx', 'main.cc', 'main.c',
            'node_main.cpp', 'app_main.cpp', 'controller_main.cpp',
            'driver_main.cpp', 'server_main.cpp', 'client_main.cpp'
        ]
        
        is_main_named = any(pattern in file_name for pattern in main_file_patterns)
        
        # Vérifier la présence de la fonction main()
        has_main_function = bool(re.search(r'int\s+main\s*\(', content))
        
        # Vérifier si c'est probablement un fichier de test
        is_test_file = any(test in file_name for test in ['test_', '_test', 'test.cpp', 'test.cxx'])
        
        # Un fichier est considéré comme main si:
        # 1. Il a une fonction main() ET n'est pas un test
        # 2. OU il a un nom de main typique ET n'est pas un test
        return (has_main_function and not is_test_file) or (is_main_named and not is_test_file)
    
    def _is_python_main_file(self, file_path: str, content: str) -> bool:
        """Détermine si un fichier Python est un point d'entrée principal"""
        file_name = Path(file_path).name.lower()
        
        # Fichiers nommés comme des mains
        main_file_patterns = [
            'main.py', 'node.py', 'node_main.py', 'app.py',
            'controller.py', 'driver.py', 'server.py', 'client.py',
            'launch.py'  # Les launch files sont spéciaux
        ]
        
        is_main_named = any(pattern == file_name for pattern in main_file_patterns)
        is_launch_file = file_name.endswith('.launch.py')
        
        # Vérifier la présence du check __main__
        has_main_check = bool(re.search(r'if\s+__name__\s*==\s*["\']__main__["\']', content))
        
        # Vérifier la présence de rclpy.init()
        has_rclpy_init = bool(re.search(r'rclpy\.init\s*\(', content))
        
        # Vérifier la présence de node creation
        has_node_creation = bool(re.search(r'create_node\s*\(|Node\s*\(', content))
        
        # Un fichier Python est considéré comme main si:
        # 1. C'est un launch file -> jamais un main
        if is_launch_file:
            return False
        
        # 2. Il a le check __main__ ET rclpy.init()
        # 3. OU il a un nom de main ET rclpy.init()
        return (has_main_check and has_rclpy_init) or (is_main_named and has_rclpy_init)
    
    def _validate_python_file(self, file_path: str, content: str):
        """Validation for ROS2 Python files"""
        lines = content.split('\n')
        
        # 1. ROS2 import check - INTELLIGENT
        has_rclpy_import = 'import rclpy' in content or 'from rclpy' in content
        
        if has_rclpy_import:
            # Déterminer si c'est un point d'entrée principal
            is_main_file = self._is_python_main_file(file_path, content)
            
            # Vérifier rclpy.init seulement dans les fichiers main
            if is_main_file:
                if not re.search(r'rclpy\.init\s*\(', content):
                    self._add_issue(file_path, 1, 'ROS2_INIT_MISSING', 'error',
                                  "rclpy.init() not found - required for ROS2 initialization in main file",
                                  suggested_fix="Add 'rclpy.init()' at the start of main()")
                
                # Vérifier rclpy.shutdown() seulement dans les fichiers main
                if not re.search(r'rclpy\.shutdown\s*\(', content):
                    self._add_issue(file_path, len(lines), 'ROS2_SHUTDOWN_MISSING', 'warning',
                                  "rclpy.shutdown() not found - recommended for resource cleanup",
                                  suggested_fix="Add 'rclpy.shutdown()' before exit")
            
            # Pour les fichiers non-main, vérifier seulement les imports
            else:
                # Vérifier si Node est utilisé mais pas importé
                if 'Node' in content and 'from rclpy.node import Node' not in content:
                    if 'class' in content and 'Node' in content:  # Probable héritage
                        self._add_issue(file_path, 1, 'NODE_IMPORT_MISSING', 'warning',
                                      "Node class used but not imported",
                                      suggested_fix="Add 'from rclpy.node import Node' at the top")
        
        # 2. time.sleep() detection (avoid in ROS2)
        for i, line in enumerate(lines, 1):
            if 'time.sleep(' in line and has_rclpy_import:
                self._add_issue(file_path, i, 'BLOCKING_SLEEP', 'error',
                              "time.sleep() detected - use rclpy.create_timer() instead",
                              line.strip(),
                              "Replace with 'self.create_timer(1.0, callback)'")
        
        # 3. DÉTECTION ROSPY.RATE ET ROSPY.SLEEP (ROS1)
        for i, line in enumerate(lines, 1):
            # Détecter rospy.Rate (ROS1)
            if 'rospy.Rate(' in line or 'rospy.rate(' in line:
                self._add_issue(file_path, i, 'ROS1_RATE_DETECTED', 'error',
                              "rospy.Rate detected - ROS1 pattern, use rclpy.create_timer() in ROS2",
                              line.strip(),
                              "Replace with 'self.create_timer(0.1, callback)'")
            
            # Détecter rospy.sleep (ROS1)
            if 'rospy.sleep(' in line:
                self._add_issue(file_path, i, 'ROS1_SLEEP_DETECTED', 'error',
                              "rospy.sleep detected - ROS1 pattern, use rclpy.create_timer() in ROS2",
                              line.strip(),
                              "Replace with 'rclpy.spin_once(node)' or timer")
            
            # Détecter ros.Rate ou ros_rate (patterns génériques)
            if re.search(r'\bros[._]?rate\b', line, re.IGNORECASE):
                self._add_issue(file_path, i, 'ROS1_STYLE_RATE', 'warning',
                              "ROS1-style rate detected - migrate to ROS2 timer patterns",
                              line.strip())
        
        # 4. DÉTECTION RATE OBJECTS ET VÉRIFICATION D'UTILISATION
        for i, line in enumerate(lines, 1):
            if 'Rate(' in line and has_rclpy_import:
                rate_match = re.search(r'(\w+)\s*=\s*Rate\s*\(', line)
                if rate_match:
                    rate_var = rate_match.group(1)
                    self.rate_objects[file_path].append((i, rate_var))
                    self._check_rate_usage_python(file_path, i, rate_var, lines)
        
        # 5. Check while True loops without exit condition
        for i, line in enumerate(lines, 1):
            if 'while True:' in line and has_rclpy_import:
                found_ok = False
                for j in range(i, min(i+10, len(lines))):
                    if 'rclpy.ok()' in lines[j] or 'not rclpy.ok()' in lines[j]:
                        found_ok = True
                        break
                
                if not found_ok:
                    self._add_issue(file_path, i, 'INFINITE_LOOP', 'error',
                                  "Infinite loop without rclpy.ok() check - may not stop cleanly",
                                  line.strip(),
                                  "Add 'while rclpy.ok():' or check 'if not rclpy.ok(): break'")
        
        # 6. Check topic names
        for i, line in enumerate(lines, 1):
            # Détecter les topics déclarés (création)
            create_pub_matches = re.findall(r"create_publisher.*?\(\s*[^,]+\s*,\s*['\"](.*?)['\"]", line)
            create_sub_matches = re.findall(r"create_subscription.*?\(\s*[^,]+\s*,\s*['\"](.*?)['\"]", line)
            
            for topic in create_pub_matches:
                if topic:
                    self.declared_topics[file_path].add(topic)
                    self._validate_topic_naming(file_path, i, topic, line)
            
            for topic in create_sub_matches:
                if topic:
                    self.declared_topics[file_path].add(topic)
                    self._validate_topic_naming(file_path, i, topic, line)
            
            # Détecter les topics utilisés (abonnements/references)
            used_topic_matches = re.findall(r"['\"](/[a-zA-Z0-9_/]+)['\"]", line)
            for topic in used_topic_matches:
                if (topic not in create_pub_matches and 
                    topic not in create_sub_matches and
                    topic.startswith('/') and
                    len(topic) > 1):
                    self.used_topics[file_path].add(topic)
        
        # 7. Check exception handling
        try:
            tree = ast.parse(content)
            self._analyze_python_ast(file_path, tree, lines)
        except SyntaxError as e:
            self._add_issue(file_path, e.lineno, 'SYNTAX_ERROR', 'error',
                          f"Python syntax error: {e.msg}")
        
        # 8. Check parameters
        param_gets = re.findall(r'\.get_parameter\s*\(\s*["\'](.*?)["\']', content)
        param_declares = re.findall(r'declare_parameter\s*\(\s*["\'](.*?)["\']', content)
        
        for param in param_declares:
            self.declared_params[file_path].add(param)
        
        for param in param_gets:
            self.used_params[file_path].add(param)
            if param not in self.declared_params[file_path]:
                self._add_issue(file_path, 0, 'UNDECLARED_PARAM', 'warning',
                              f"Parameter '{param}' used but not declared in this file",
                              suggested_fix=f"Add 'self.declare_parameter(\"{param}\", default_value)'")
        
        # 9. Check for missing dependencies
        self._check_missing_imports(file_path, content, lines)
    
    def _validate_cpp_file(self, file_path: str, content: str):
        """Validation for ROS2 C++ files"""
        lines = content.split('\n')
        
        # 1. Check ROS2 includes - INTELLIGENT
        has_rclcpp_include = '#include <rclcpp/rclcpp.hpp>' in content
        uses_rclcpp = 'rclcpp::' in content
        
        if uses_rclcpp:
            # Déterminer si c'est un point d'entrée principal
            is_main_file = self._is_cpp_main_file(file_path, content)
            
            # Vérifier rclcpp::init seulement dans les fichiers main
            if is_main_file:
                if not re.search(r'rclcpp::init\s*\(', content):
                    self._add_issue(file_path, 1, 'ROS2_CPP_INIT_MISSING', 'error',
                                  "rclcpp::init() not found - required for ROS2 initialization in main file",
                                  suggested_fix="Add 'rclcpp::init(argc, argv);' at the start of main()")
                
                # Vérifier rclcpp::shutdown seulement dans les fichiers main
                if not re.search(r'rclcpp::shutdown\s*\(', content):
                    self._add_issue(file_path, len(lines), 'ROS2_CPP_SHUTDOWN_MISSING', 'warning',
                                  "rclcpp::shutdown() not found - recommended for resource cleanup",
                                  suggested_fix="Add 'rclcpp::shutdown();' before return in main()")
            
            # Pour les headers, vérifier seulement les includes
            elif file_path.endswith(('.h', '.hpp', '.hxx')):
                if not has_rclcpp_include and uses_rclcpp:
                    self._add_issue(file_path, 1, 'RCLCPP_HEADER_INCLUDE_MISSING', 'warning',
                                  "rclcpp used in header but not included",
                                  suggested_fix="Add '#include <rclcpp/rclcpp.hpp>' at the top")
                
                # Vérifier les guard d'inclusion pour les headers
                self._validate_cpp_header_guards(file_path, content, lines)
        
        # 2. sleep() or usleep() detection (avoid)
        for i, line in enumerate(lines, 1):
            if any(sleep in line for sleep in ['sleep(', 'usleep(', 'std::this_thread::sleep']):
                if uses_rclcpp:
                    self._add_issue(file_path, i, 'BLOCKING_SLEEP_CPP', 'error',
                                  "sleep() detected - use rclcpp::Rate or create_wall_timer()",
                                  line.strip(),
                                  "Replace with 'rclcpp::Rate rate(10); rate.sleep();' or 'create_wall_timer()'")
        
        # 3. DÉTECTION ROS::RATE (ROS1) dans C++
        for i, line in enumerate(lines, 1):
            # Détecter ros::Rate (ROS1)
            if 'ros::Rate' in line:
                self._add_issue(file_path, i, 'ROS1_CPP_RATE_DETECTED', 'error',
                              "ros::Rate detected - ROS1 pattern, use rclcpp::Rate or create_wall_timer() in ROS2",
                              line.strip(),
                              "Replace with 'rclcpp::Rate' or 'create_wall_timer()'")
            
            # Détecter ros::Duration (ROS1)
            if 'ros::Duration' in line and uses_rclcpp:
                self._add_issue(file_path, i, 'ROS1_DURATION_DETECTED', 'warning',
                              "ros::Duration detected - use rclcpp::Duration in ROS2",
                              line.strip(),
                              "Replace with 'rclcpp::Duration'")
        
        # 4. DÉTECTION RATE OBJECTS C++ ET VÉRIFICATION
        for i, line in enumerate(lines, 1):
            # Détecter rclcpp::Rate (bonne pratique ROS2)
            if 'rclcpp::Rate' in line:
                rate_match = re.search(r'rclcpp::Rate\s+(\w+)\s*\(', line)
                if rate_match:
                    rate_var = rate_match.group(1)
                    self.rate_objects[file_path].append((i, rate_var))
                    self._check_rate_usage_cpp(file_path, i, rate_var, lines)
        
        # 5. Check rclcpp::ok() in loops
        for i, line in enumerate(lines, 1):
            if 'while (' in line and 'true' in line.lower() and uses_rclcpp:
                found_ok = False
                for j in range(i, min(i+10, len(lines))):
                    if 'rclcpp::ok()' in lines[j] or '!rclcpp::ok()' in lines[j]:
                        found_ok = True
                        break
                
                if not found_ok:
                    self._add_issue(file_path, i, 'INFINITE_LOOP_CPP', 'error',
                                  "Infinite loop without rclcpp::ok() check",
                                  line.strip(),
                                  "Add 'while (rclcpp::ok())' or check 'if (!rclcpp::ok()) break'")
        
        # 6. Check raw pointers (bad practice)
        for i, line in enumerate(lines, 1):
            if 'new ' in line and 'shared_ptr' not in line and 'unique_ptr' not in line:
                self._add_issue(file_path, i, 'RAW_POINTER', 'warning',
                              "Use of 'new' - prefer smart pointers",
                              line.strip(),
                              "Replace with 'std::make_shared<T>()' or 'std::make_unique<T>()'")
        
        # 7. Check topics
        for i, line in enumerate(lines, 1):
            # Detect create_publisher/create_subscription
            create_pub_matches = re.findall(r'create_publisher.*?\(\s*[^,]+\s*,\s*["\'](.*?)["\']', line)
            create_sub_matches = re.findall(r'create_subscription.*?\(\s*[^,]+\s*,\s*["\'](.*?)["\']', line)
            
            for topic in create_pub_matches:
                if topic:
                    self.declared_topics[file_path].add(topic)
                    self._validate_topic_naming(file_path, i, topic, line)
            
            for topic in create_sub_matches:
                if topic:
                    self.declared_topics[file_path].add(topic)
                    self._validate_topic_naming(file_path, i, topic, line)
            
            # Détecter les topics utilisés
            used_topic_matches = re.findall(r'["\'](/[a-zA-Z0-9_/]+)["\']', line)
            for topic in used_topic_matches:
                if (topic not in create_pub_matches and 
                    topic not in create_sub_matches and
                    topic.startswith('/') and
                    len(topic) > 1):
                    self.used_topics[file_path].add(topic)
        
        # 8. Check parameters in C++
        param_gets = re.findall(r'get_parameter\s*\(\s*["\'](.*?)["\']', content)
        param_declares = re.findall(r'declare_parameter\s*\(\s*["\'](.*?)["\']', content)
        
        for param in param_declares:
            self.declared_params[file_path].add(param)
        
        for param in param_gets:
            self.used_params[file_path].add(param)
            if param not in self.declared_params[file_path]:
                self._add_issue(file_path, 0, 'UNDECLARED_PARAM_CPP', 'warning',
                              f"Parameter '{param}' used but not declared in this file",
                              suggested_fix=f"Add 'this->declare_parameter(\"{param}\", default_value);'")
    
    def _validate_cpp_header_guards(self, file_path: str, content: str, lines: List[str]):
        """Valide les guard d'inclusion pour les headers C++"""
        file_name = Path(file_path).name.upper().replace('.', '_').replace('-', '_')
        guard_name = f"{file_name}_"
        
        # Chercher les guards
        found_ifndef = False
        found_define = False
        found_endif = False
        
        for i, line in enumerate(lines, 1):
            if f'#ifndef {guard_name}' in line or f'#ifndef _{guard_name}' in line:
                found_ifndef = True
            if f'#define {guard_name}' in line or f'#define _{guard_name}' in line:
                found_define = True
            if f'#endif // {guard_name}' in line or f'#endif  // {guard_name}' in line:
                found_endif = True
        
        if not found_ifndef or not found_define:
            self._add_issue(file_path, 1, 'MISSING_INCLUDE_GUARD', 'warning',
                          f"Missing or incomplete include guard for header file",
                          suggested_fix=f"Add:\n#ifndef {guard_name}\n#define {guard_name}\n\n// content\n\n#endif // {guard_name}")
    
    def _check_rate_usage_python(self, file_path: str, line_no: int, rate_var: str, lines: List[str]):
        """Vérifie l'utilisation correcte d'un objet Rate en Python"""
        found_sleep = False
        start_line = max(0, line_no - 1)
        
        for i in range(start_line, min(line_no + 20, len(lines))):
            if f'{rate_var}.sleep()' in lines[i]:
                found_sleep = True
                self._check_rate_in_loop(file_path, i, rate_var, lines)
                break
        
        if not found_sleep:
            self._add_issue(file_path, line_no, 'RATE_NOT_USED', 'warning',
                          f"Rate object '{rate_var}' declared but sleep() not called nearby",
                          lines[line_no-1] if line_no <= len(lines) else "",
                          f"Call '{rate_var}.sleep()' in a loop or remove the Rate object")
    
    def _check_rate_usage_cpp(self, file_path: str, line_no: int, rate_var: str, lines: List[str]):
        """Vérifie l'utilisation correcte d'un objet Rate en C++"""
        found_sleep = False
        start_line = max(0, line_no - 1)
        
        for i in range(start_line, min(line_no + 20, len(lines))):
            if f'{rate_var}.sleep()' in lines[i]:
                found_sleep = True
                self._check_rate_in_loop_cpp(file_path, i, rate_var, lines)
                break
        
        if not found_sleep:
            self._add_issue(file_path, line_no, 'RATE_NOT_USED_CPP', 'warning',
                          f"Rate object '{rate_var}' declared but sleep() not called nearby",
                          lines[line_no-1] if line_no <= len(lines) else "",
                          f"Call '{rate_var}.sleep();' in a loop or remove the Rate object")
    
    def _check_rate_in_loop(self, file_path: str, line_no: int, rate_var: str, lines: List[str]):
        """Vérifie si le rate.sleep() est dans une boucle"""
        in_loop = False
        for i in range(max(0, line_no - 10), line_no):
            if 'while ' in lines[i] or 'for ' in lines[i]:
                in_loop = True
                break
        
        if not in_loop:
            self._add_issue(file_path, line_no, 'RATE_OUTSIDE_LOOP', 'warning',
                          f"rate.sleep() called outside of loop - may block indefinitely",
                          lines[line_no-1] if line_no <= len(lines) else "",
                          "Move rate.sleep() inside a while or for loop")
    
    def _check_rate_in_loop_cpp(self, file_path: str, line_no: int, rate_var: str, lines: List[str]):
        """Vérifie si le rate.sleep() est dans une boucle en C++"""
        in_loop = False
        for i in range(max(0, line_no - 10), line_no):
            if 'while ' in lines[i] or 'for ' in lines[i]:
                in_loop = True
                break
        
        if not in_loop:
            self._add_issue(file_path, line_no, 'RATE_OUTSIDE_LOOP_CPP', 'warning',
                          f"rate.sleep() called outside of loop - may block indefinitely",
                          lines[line_no-1] if line_no <= len(lines) else "",
                          "Move rate.sleep() inside a while or for loop")
    
    def _check_missing_imports(self, file_path: str, content: str, lines: List[str]):
        """Vérifie les imports manquants"""
        ros2_patterns = [
            ('rclpy\.', 'import rclpy'),
            ('std_msgs\.msg', 'from std_msgs.msg import'),
            ('geometry_msgs\.msg', 'from geometry_msgs.msg import'),
            ('sensor_msgs\.msg', 'from sensor_msgs.msg import'),
            ('nav_msgs\.msg', 'from nav_msgs.msg import'),
        ]
        
        for pattern, required_import in ros2_patterns:
            if re.search(pattern, content) and required_import not in content:
                self._add_issue(file_path, 1, 'MISSING_IMPORT', 'warning',
                              f"Missing import: '{required_import}'",
                              suggested_fix=f"Add '{required_import}' at the top of the file")
    
    def _validate_launch_file(self, file_path: str, content: str):
        """Validation for launch files"""
        lines = content.split('\n')
        
        if file_path.endswith('.xml'):
            self._validate_xml_launch(file_path, content)
        elif file_path.endswith('.py'):
            self._validate_python_launch(file_path, content, lines)
    
    def _validate_xml_launch(self, file_path: str, content: str):
        """Validation for XML launch files"""
        try:
            root = ET.fromstring(content)
            
            for node in root.findall('.//node'):
                pkg = node.get('pkg')
                type_attr = node.get('type')
                name = node.get('name')
                
                if not pkg:
                    self._add_issue(file_path, 0, 'LAUNCH_NO_PKG', 'error',
                                  f"Node '{name}' without 'pkg' attribute",
                                  suggested_fix=f"Add pkg='package_name' to node")
                if not type_attr:
                    self._add_issue(file_path, 0, 'LAUNCH_NO_TYPE', 'error',
                                  f"Node '{name}' without 'type' attribute",
                                  suggested_fix=f"Add type='executable_name' to node")
                if not name:
                    self._add_issue(file_path, 0, 'LAUNCH_NO_NAME', 'error',
                                  "Node without 'name' attribute",
                                  suggested_fix="Add name='node_name' to node")
                
                if name and not self._is_valid_node_name(name):
                    self._add_issue(file_path, 0, 'NODE_NAMING', 'warning',
                                  f"Node name '{name}' doesn't follow ROS2 conventions",
                                  suggested_fix=f"Change to snake_case: '{name.lower().replace('-', '_')}'")
            
            for param in root.findall('.//param'):
                name = param.get('name')
                if name and name.startswith('~'):
                    self._add_issue(file_path, 0, 'PRIVATE_PARAM_SYNTAX', 'warning',
                                  f"Parameter '{name}' uses '~' - ROS1 syntax, use $(env) for ROS2",
                                  suggested_fix=f"Replace '~{name[1:]}' with '$(env VAR_NAME)' or remove '~'")
        
        except ET.ParseError as e:
            self._add_issue(file_path, e.position[0] if hasattr(e, 'position') else 0,
                          'XML_SYNTAX_ERROR', 'error', f"XML error: {e.msg}")
    
    def _validate_python_launch(self, file_path: str, content: str, lines: List[str]):
        """Validation for Python launch files"""
        if 'from launch import' not in content or 'from launch_ros.actions import Node' not in content:
            self._add_issue(file_path, 1, 'LAUNCH_IMPORTS', 'warning',
                          "Missing or incomplete launch/launch_ros imports",
                          suggested_fix="Add: 'from launch import LaunchDescription\nfrom launch_ros.actions import Node'")
        
        for i, line in enumerate(lines, 1):
            if 'rospy' in line or 'ROSLaunch' in line:
                self._add_issue(file_path, i, 'ROS1_SYNTAX', 'warning',
                              "ROS1 syntax detected - migrate to ROS2",
                              line.strip(),
                              "Replace with ROS2 launch API: 'from launch_ros.actions import Node'")
    
    def _validate_yaml_file(self, file_path: str, content: str):
        """Validation for YAML files (params)"""
        try:
            import yaml
            data = yaml.safe_load(content)
            
            if isinstance(data, dict):
                for key, value in data.items():
                    if isinstance(value, dict):
                        if 'ros__parameters' in value:
                            params = value['ros__parameters']
                            if isinstance(params, dict):
                                for param_name in params.keys():
                                    if not self._is_valid_param_name(param_name):
                                        self._add_issue(file_path, 0, 'PARAM_NAMING', 'warning',
                                                      f"Parameter name '{param_name}' not conventional",
                                                      suggested_fix=f"Change to snake_case: '{param_name.lower().replace('-', '_')}'")
        
        except yaml.YAMLError as e:
            self._add_issue(file_path, 0, 'YAML_SYNTAX', 'error',
                          f"YAML error: {str(e)}")
        except ImportError:
            self._add_issue(file_path, 0, 'YAML_MODULE_MISSING', 'warning',
                          "PyYAML not installed - YAML validation skipped")
    
    def _validate_urdf_file(self, file_path: str, content: str):
        """Validation for URDF/XACRO files"""
        lines = content.split('\n')
        
        if '<robot' not in content:
            self._add_issue(file_path, 1, 'URDF_NO_ROBOT', 'error',
                          "Missing <robot> tag",
                          suggested_fix="Add '<robot name=\"robot_name\">' at the top and '</robot>' at the bottom")
        
        link_pattern = r'<link\s+name=["\'](.*?)["\']'
        joint_pattern = r'<joint\s+name=["\'](.*?)["\']'
        
        for i, line in enumerate(lines, 1):
            link_matches = re.findall(link_pattern, line)
            for link in link_matches:
                if link and not self._is_valid_tf_frame(link):
                    self._add_issue(file_path, i, 'TF_FRAME_NAMING', 'warning',
                                  f"TF frame name '{link}' not conventional",
                                  line.strip(),
                                  f"Change to snake_case: '{link.lower().replace('-', '_')}'")
            
            joint_matches = re.findall(joint_pattern, line)
            for joint in joint_matches:
                if joint and not self._is_valid_joint_name(joint):
                    self._add_issue(file_path, i, 'JOINT_NAMING', 'warning',
                                  f"Joint name '{joint}' not conventional",
                                  line.strip(),
                                  f"Change to snake_case ending with '_joint': '{joint.lower().replace('-', '_')}_joint'")
    
    def _analyze_python_ast(self, file_path: str, tree: ast.AST, lines: List[str]):
        """AST analysis to detect logical issues"""
        
        class Analyzer(ast.NodeVisitor):
            def __init__(self, validator, file_path, lines):
                self.validator = validator
                self.file_path = file_path
                self.lines = lines
                self.in_try_block = False
            
            def visit_Try(self, node):
                self.in_try_block = True
                if not node.handlers:
                    line_no = node.lineno
                    self.validator._add_issue(
                        self.file_path, line_no, 'BARE_TRY', 'error',
                        "Try block without except - unhandled exception",
                        self.lines[line_no-1] if line_no <= len(self.lines) else "",
                        "Add 'except Exception as e:' or specific exception handler"
                    )
                self.generic_visit(node)
                self.in_try_block = False
            
            def visit_Raise(self, node):
                line_no = node.lineno
                if not self.in_try_block:
                    self.validator._add_issue(
                        self.file_path, line_no, 'UNHANDLED_RAISE', 'warning',
                        "Potentially unhandled raise",
                        self.lines[line_no-1] if line_no <= len(self.lines) else "",
                        "Wrap in try-except block or ensure it's caught upstream"
                    )
                self.generic_visit(node)
        
        analyzer = Analyzer(self, file_path, lines)
        analyzer.visit(tree)
    
    def _validate_topic_naming(self, file_path: str, line_no: int, topic: str, line: str):
        """Validates topic naming conventions"""
        if not topic.startswith('/'):
            self._add_issue(file_path, line_no, 'TOPIC_NO_SLASH', 'warning',
                          f"Topic '{topic}' doesn't start with '/'",
                          line.strip(),
                          f"Change to '/{topic}'")
        
        if not re.match(r'^/[a-zA-Z0-9_/]+$', topic):
            self._add_issue(file_path, line_no, 'TOPIC_INVALID_CHARS', 'warning',
                          f"Topic '{topic}' contains invalid characters",
                          line.strip(),
                          "Only use letters, numbers, underscores and slashes")
        
        if re.search(r'[A-Z]', topic.replace('/', '')):
            if not any(topic.lower() == ct.lower() for ct in self.common_ros_topics):
                self._add_issue(file_path, line_no, 'TOPIC_CASING', 'info',
                              f"Topic '{topic}' contains uppercase - snake_case recommended",
                              line.strip(),
                              f"Change to lowercase: '{topic.lower()}'")
        
        if len(topic) > 100:
            self._add_issue(file_path, line_no, 'TOPIC_TOO_LONG', 'warning',
                          f"Topic '{topic}' too long ({len(topic)} chars)",
                          line.strip(),
                          "Shorten topic name to less than 100 characters")
    
    def _is_valid_node_name(self, name: str) -> bool:
        """Checks if a node name is valid"""
        return bool(re.match(r'^[a-z][a-z0-9_]*$', name))
    
    def _is_valid_param_name(self, name: str) -> bool:
        """Checks if a parameter name is valid"""
        return bool(re.match(r'^[a-z][a-z0-9_]*$', name))
    
    def _is_valid_tf_frame(self, frame: str) -> bool:
        """Checks if a TF frame name is valid"""
        return bool(re.match(r'^[a-z][a-z0-9_]*$', frame))
    
    def _is_valid_joint_name(self, joint: str) -> bool:
        """Checks if a joint name is valid"""
        return bool(re.match(r'^[a-z][a-z0-9_]*_joint$', joint))
    
    def _extract_package_dependencies(self, file_path: str, content: str):
        """Extrait les dépendances depuis package.xml"""
        try:
            root = ET.fromstring(content)
            package_name = root.find('name')
            if package_name is not None:
                pkg_name = package_name.text
                
                dependencies = set()
                for dep_type in ['build_depend', 'exec_depend', 'depend', 'test_depend']:
                    for dep in root.findall(f'.//{dep_type}'):
                        if dep.text:
                            dependencies.add(dep.text)
                
                self.package_dependencies[pkg_name] = dependencies
                
                for dep in dependencies:
                    if 'roscpp' in dep or 'rospy' in dep or 'roslib' in dep:
                        self._add_issue(file_path, 0, 'ROS1_DEPENDENCY', 'error',
                                      f"ROS1 dependency '{dep}' detected - migrate to ROS2 equivalents",
                                      suggested_fix=f"Replace '{dep}' with ROS2 equivalent (rclcpp, rclpy, etc.)")
                
        except ET.ParseError as e:
            self._add_issue(file_path, 0, 'XML_PARSE_ERROR', 'error',
                          f"Cannot parse package.xml: {str(e)}")
    
    def validate_cross_references(self, workspace_path: str):
        """Vérifie les références croisées entre fichiers"""
        
        all_declared_params = set()
        all_used_params = set()
        
        for file_path in self.declared_params:
            all_declared_params.update(self.declared_params[file_path])
        
        for file_path in self.used_params:
            all_used_params.update(self.used_params[file_path])
        
        undeclared_globally = all_used_params - all_declared_params
        for param in undeclared_globally:
            self._add_issue("GLOBAL", 0, 'GLOBAL_UNDECLARED_PARAM', 'error',
                          f"Parameter '{param}' is used but never declared in any file",
                          suggested_fix=f"Declare parameter '{param}' in a node's constructor")
        
        all_declared_topics = set()
        all_used_topics = set()
        
        for file_path in self.declared_topics:
            all_declared_topics.update(self.declared_topics[file_path])
        
        for file_path in self.used_topics:
            all_used_topics.update(self.used_topics[file_path])
        
        topics_without_publisher = all_used_topics - all_declared_topics
        for topic in topics_without_publisher:
            if not topic.startswith(('/tf', '/tf_static', '/rosout')):
                self._add_issue("GLOBAL", 0, 'TOPIC_WITHOUT_PUBLISHER', 'warning',
                              f"Topic '{topic}' is subscribed to but no publisher found",
                              suggested_fix=f"Add publisher for topic '{topic}' or check topic name")
        
        topics_without_subscriber = all_declared_topics - all_used_topics
        for topic in topics_without_subscriber:
            if not topic.startswith(('/tf', '/tf_static', '/rosout')):
                self._add_issue("GLOBAL", 0, 'TOPIC_WITHOUT_SUBSCRIBER', 'info',
                              f"Topic '{topic}' is published but no subscriber found",
                              suggested_fix=f"Add subscriber for topic '{topic}' or remove publisher if not needed")
    
    def check_real_topics_existence(self):
        """Vérifie l'existence réelle des topics dans un système ROS2 actif"""
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                active_topics = set(result.stdout.strip().split('\n'))
                
                all_expected_topics = set()
                for topics in self.declared_topics.values():
                    all_expected_topics.update(topics)
                for topics in self.used_topics.values():
                    all_expected_topics.update(topics)
                
                missing_in_real = all_expected_topics - active_topics
                for topic in missing_in_real:
                    if topic:
                        self._add_issue("SYSTEM", 0, 'TOPIC_NOT_ACTIVE', 'warning',
                                      f"Topic '{topic}' declared but not active in ROS2 system",
                                      suggested_fix="Ensure node is running and topic name is correct")
        
        except (subprocess.SubprocessError, ImportError):
            pass
    
    def _add_issue(self, file_path: str, line_number: int, issue_type: str, 
                   severity: str, message: str, code_snippet: str = "", 
                   suggested_fix: str = ""):
        """Adds a detected issue with optional suggested fix"""
        self.issues.append(ValidationIssue(
            file_path=file_path,
            line_number=line_number,
            issue_type=issue_type,
            severity=severity,
            message=message,
            code_snippet=code_snippet[:200],
            suggested_fix=suggested_fix
        ))
    
    def get_summary(self) -> Dict[str, Any]:
        """Returns a summary of found issues"""
        summary = {
            'total_issues': len(self.issues),
            'errors': len([i for i in self.issues if i.severity == 'error']),
            'warnings': len([i for i in self.issues if i.severity == 'warning']),
            'infos': len([i for i in self.issues if i.severity == 'info']),
            'by_type': {},
            'files_affected': len(set(i.file_path for i in self.issues)),
            'topics_declared': len(set().union(*self.declared_topics.values())),
            'topics_used': len(set().union(*self.used_topics.values())),
            'params_declared': len(set().union(*self.declared_params.values())),
            'params_used': len(set().union(*self.used_params.values())),
        }
        
        for issue in self.issues:
            summary['by_type'][issue.issue_type] = summary['by_type'].get(issue.issue_type, 0) + 1
        
        return summary

def validate_workspace(root_path: str) -> Dict[str, Any]:
    """Validates a complete ROS2 workspace"""
    from pathlib import Path
    
    all_issues = []
    validator_instance = ROS2Validator()
    
    for file_path in Path(root_path).rglob('*'):
        if file_path.is_file():
            if (file_path.suffix in ['.py', '.cpp', '.hpp', '.c', '.h', '.launch', 
                                   '.launch.py', '.launch.xml', '.yaml', '.yml', 
                                   '.urdf', '.xacro'] or 
                file_path.name == 'package.xml'):
                
                if not any(part.startswith('.') or part in ['build', 'install', 'log'] 
                          for part in file_path.parts):
                    
                    try:
                        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                            content = f.read()
                        
                        issues = validator_instance.validate_file(str(file_path), content)
                        all_issues.extend(issues)
                        
                    except Exception as e:
                        validator_instance._add_issue(
                            str(file_path), 0, 'FILE_READ_ERROR', 'error',
                            f"Unable to read file: {str(e)}"
                        )
    
    validator_instance.validate_cross_references(root_path)
    
    issues_dict = [
        {
            'file_path': issue.file_path,
            'line_number': issue.line_number,
            'issue_type': issue.issue_type,
            'severity': issue.severity,
            'message': issue.message,
            'code_snippet': issue.code_snippet,
            'suggested_fix': issue.suggested_fix
        }
        for issue in all_issues
    ]
    
    return {
        'issues': issues_dict,
        'summary': validator_instance.get_summary(),
        'validation_passed': len([i for i in all_issues if i.severity == 'error']) == 0
    }

def validate_single_file(file_path: str) -> Dict[str, Any]:
    """Valide un seul fichier"""
    validator_instance = ROS2Validator()
    
    try:
        with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
        
        issues = validator_instance.validate_file(file_path, content)
        
        issues_dict = [
            {
                'file_path': issue.file_path,
                'line_number': issue.line_number,
                'issue_type': issue.issue_type,
                'severity': issue.severity,
                'message': issue.message,
                'code_snippet': issue.code_snippet,
                'suggested_fix': issue.suggested_fix
            }
            for issue in issues
        ]
        
        return {
            'issues': issues_dict,
            'summary': validator_instance.get_summary(),
            'validation_passed': len([i for i in issues if i.severity == 'error']) == 0
        }
        
    except Exception as e:
        return {
            'issues': [{
                'file_path': file_path,
                'line_number': 0,
                'issue_type': 'FILE_ERROR',
                'severity': 'error',
                'message': f"Cannot read file: {str(e)}",
                'code_snippet': '',
                'suggested_fix': ''
            }],
            'summary': {'total_issues': 1, 'errors': 1, 'warnings': 0, 'infos': 0},
            'validation_passed': False
        }

if __name__ == "__main__":
    """Exemple d'utilisation en ligne de commande"""
    import argparse
    
    parser = argparse.ArgumentParser(description="ROS2 Validator")
    parser.add_argument("path", help="Chemin du fichier ou workspace à valider")
    parser.add_argument("--format", choices=["json", "text"], default="text", 
                       help="Format de sortie")
    parser.add_argument("--single-file", action="store_true", 
                       help="Valider un seul fichier")
    
    args = parser.parse_args()
    
    if args.single_file:
        result = validate_single_file(args.path)
    else:
        result = validate_workspace(args.path)
    
    if args.format == "json":
        print(json.dumps(result, indent=2))
    else:
        print(f"Validation Results:")
        print(f"Total Issues: {result['summary']['total_issues']}")
        print(f"Errors: {result['summary']['errors']}")
        print(f"Warnings: {result['summary']['warnings']}")
        print(f"Info: {result['summary']['infos']}")
        print(f"Validation {'PASSED' if result['validation_passed'] else 'FAILED'}")
        
        if result['issues']:
            print("\nIssues:")
            for issue in result['issues']:
                print(f"\n[{issue['severity'].upper()}] {issue['file_path']}:{issue['line_number']}")
                print(f"  Type: {issue['issue_type']}")
                print(f"  Message: {issue['message']}")
                if issue['code_snippet']:
                    print(f"  Code: {issue['code_snippet']}")
                if issue['suggested_fix']:
                    print(f"  Fix: {issue['suggested_fix']}")