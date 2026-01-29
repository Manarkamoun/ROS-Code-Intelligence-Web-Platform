from fastapi import FastAPI, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
import zipfile
import os
import shutil
import uuid
import re
import yaml
import json
from typing import List, Dict, Any, Optional
from pathlib import Path
from collections import defaultdict
import networkx as nx
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import io
import base64
import ast
from validator import validate_workspace, ROS2Validator 
import xml.etree.ElementTree as ET

app = FastAPI(title="ROS2 Advanced Auditor")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ===== PROFESSIONAL ANALYSIS FUNCTIONS =====

def generate_architecture_mermaid(nodes: List[Dict]) -> str:
    """Generate Mermaid code for system architecture visualization."""
    # First, classify nodes into layers
    layers = classify_architecture_layers(nodes)
    
    # DEBUG: Print layer contents
    print("\n=== DEBUG: Layer Contents ===")
    for layer_name, layer_nodes in layers.items():
        print(f"\n{layer_name.upper()} ({len(layer_nodes)} nodes):")
        for i, node in enumerate(layer_nodes[:3]):  # Show first 3 nodes
            print(f"  {i+1}. {node.get('name', 'unknown')}")
    
    mermaid_code = "graph TB\n"
    
    # Define layer symbols and colors
    layer_symbols = {
        "perception": "👁️",
        "planning": "🧠", 
        "control": "🎮",
        "execution": "⚙️",
        "supervision": "👑",
        "infrastructure": "🔧",
        "other": "📦"
    }
    
    layer_colors = {
        "perception": "#1e3a8a",
        "planning": "#065f46",
        "control": "#92400e",
        "execution": "#5b21b6",
        "supervision": "#9d174d",
        "infrastructure": "#374151",
        "other": "#64748b"
    }
    
    # Create subgraphs for each layer
    for layer_name, layer_nodes in layers.items():
        if layer_nodes:  # Only create subgraph if there are nodes
            mermaid_code += f'    subgraph {layer_name}["{layer_symbols.get(layer_name, "📦")} {layer_name.upper()}"]\n'
            
            # Add nodes to subgraph (limit to 5 for readability)
            for i, node in enumerate(layer_nodes[:5]):
                node_name = node.get('name', 'unknown')
                # Create safe ID for mermaid
                node_id = f"{layer_name}_{i}_{node_name.replace(' ', '_').replace('.', '_')}"
                mermaid_code += f'        {node_id}["{node_name}"]\n'
            
            mermaid_code += "    end\n\n"
    
    # Add inter-layer connections
    mermaid_code += "    %% Architecture Connections\n"
    mermaid_code += "    perception --> planning\n"
    mermaid_code += "    planning --> control\n"
    mermaid_code += "    control --> execution\n"
    mermaid_code += "    supervision --> planning\n"
    mermaid_code += "    supervision --> control\n"
    mermaid_code += "    infrastructure --> perception\n"
    mermaid_code += "    infrastructure --> planning\n"
    mermaid_code += "    infrastructure --> control\n"
    mermaid_code += "    infrastructure --> execution\n"
    mermaid_code += "    infrastructure --> supervision\n\n"
    
    # Add style definitions
    for layer_name, color in layer_colors.items():
        mermaid_code += f'    classDef {layer_name} fill:{color},stroke:{color},color:#fff\n'
    
    # Apply styles to subgraphs
    for layer_name in layers.keys():
        if layers[layer_name]:  # Only apply if layer has nodes
            mermaid_code += f'    class {layer_name} {layer_name}\n'
    
    return mermaid_code

def classify_architecture_layers(nodes: List[Dict]) -> Dict[str, List[Dict]]:
    """Classifie les nœuds dans des couches architecturales avec fallback vers 'other'."""
    layers = {
        "perception": [], "planning": [], "control": [],
        "execution": [], "supervision": [], "infrastructure": [], "other": []
    }
    
    # Définir les mots-clés pour chaque couche
    perception_keywords = ['sensor', 'camera', 'lidar', 'imu', 'laser', 'image', 'scan', 'pointcloud', 
                          'depth', 'rgbd', 'stereo', 'vision', 'detection', 'optical', 'ir', 'thermal']
    
    planning_keywords = ['plan', 'planner', 'trajectory', 'path', 'nav', 'goal', 'bt_', 'behavior', 
                        'move_base', 'navigation', 'costmap', 'global', 'local', 'slam', 'mapping']
    
    control_keywords = ['controller', 'pid', 'motor', 'cmd_vel', 'twist', 'actuator', 'drive', 
                       'velocity', 'effort', 'torque', 'steering', 'servo', 'pwm', 'hbridge']
    
    execution_keywords = ['driver', 'hardware', 'interface', 'can', 'serial', 'gpio', 'arduino', 
                         'stm32', 'i2c', 'spi', 'uart', 'rosserial', 'firmware', 'device']
    
    supervision_keywords = ['manager', 'supervisor', 'monitor', 'health', 'diagnostic', 'safety', 
                           'watchdog', 'recovery', 'fault', 'emergency', 'diagnostics', 'status']
    
    infrastructure_keywords = ['tf_publisher', 'state_publisher', 'bridge', 'relay', 'recorder', 
                              'logger', 'bag', 'rosbag', 'transform', 'static_transform', 'remap']
    
    for node in nodes:
        # 1. Extraire et normaliser les données
        name = str(node.get('name', '')).lower()
        pubs = [str(p).lower() for p in node.get('pubs', [])]
        subs = [str(s).lower() for s in node.get('subs', [])]
        srvs = [str(s).lower() for s in node.get('srvs', [])]
        acts = [str(a).lower() for a in node.get('acts', [])]
        
        # Créer une chaîne de recherche complète
        all_content = " ".join([name] + pubs + subs + srvs + acts)
        
        # DEBUG: Afficher le contenu pour le débogage
        # print(f"DEBUG: Node '{name}' content: {all_content[:100]}...")
        
        # 2. Classification hiérarchique
        classified = False
        
        # Vérifier chaque couche dans l'ordre
        if any(kw in all_content for kw in perception_keywords):
            layers["perception"].append(node)
            classified = True
            # print(f"  → Classified as PERCEPTION")
        
        elif any(kw in all_content for kw in planning_keywords):
            layers["planning"].append(node)
            classified = True
            # print(f"  → Classified as PLANNING")
            
        elif any(kw in all_content for kw in control_keywords):
            layers["control"].append(node)
            classified = True
            # print(f"  → Classified as CONTROL")
            
        elif any(kw in all_content for kw in execution_keywords):
            layers["execution"].append(node)
            classified = True
            # print(f"  → Classified as EXECUTION")
            
        elif any(kw in all_content for kw in supervision_keywords):
            layers["supervision"].append(node)
            classified = True
            # print(f"  → Classified as SUPERVISION")

        elif any(kw in all_content for kw in infrastructure_keywords):
            layers["infrastructure"].append(node)
            classified = True
            # print(f"  → Classified as INFRASTRUCTURE")

        # 3. Fallback vers 'other' si non classifié
        if not classified:
            layers["other"].append(node)
            # print(f"  → Classified as OTHER (no keywords matched)")
    
    # Afficher le résumé de la classification
    print("\n=== Classification Summary ===")
    for layer_name, layer_nodes in layers.items():
        print(f"{layer_name.upper()}: {len(layer_nodes)} nodes")
    
    return layers

def perform_behavioral_analysis(nodes: List[Dict]) -> Dict[str, Any]:
    """
    Professional behavioral analysis focusing on decision-making patterns,
    architecture layers, and communication chains.
    """
    
    # 1. Frequency Analysis
    frequency_analysis = []
    for node in nodes:
        hz = node.get('hz', 0)
        category = categorize_frequency(hz)
        
        frequency_analysis.append({
            "node_name": node.get('name', 'unknown'),
            "frequency_hz": hz,
            "category": category,
            "criticality": determine_criticality(hz),
            "package": node.get('package', 'unknown'),
            "publication_rate": calculate_publication_rate(node)
        })
    
    # 2. Decision-Making Analysis
    decision_analysis = []
    for node in nodes:
        complexity = analyze_decision_complexity(node)
        decision_analysis.append({
            "node_name": node.get('name', 'unknown'),
            "decision_score": complexity['score'],
            "decision_level": complexity['level'],
            "input_diversity": len(set(node.get('subs', []))),
            "output_diversity": len(set(node.get('pubs', []))),
            "service_responsibility": len(node.get('srvs', [])),
            "action_responsibility": len(node.get('acts', [])),
            "cognitive_load": calculate_cognitive_load(node)
        })
    
    # 3. Architecture Layer Classification (APPEL UNIQUE)
    architecture_layers = classify_architecture_layers(nodes)
    
    # 4. Communication Chain Analysis
    communication_chains = analyze_communication_chains(nodes)
    
    # 5. Longest Chain Detection
    longest_chain = find_longest_communication_chain(nodes)
    
    return {
        "frequency_analysis": sorted(frequency_analysis, key=lambda x: x['frequency_hz'], reverse=True),
        "decision_analysis": sorted(decision_analysis, key=lambda x: x['decision_score'], reverse=True),
        "architecture_layers": architecture_layers,
        "communication_chains": communication_chains,
        "longest_chain": longest_chain,
        "summary": {
            "total_nodes_analyzed": len(nodes),
            "high_frequency_nodes": len([f for f in frequency_analysis if f['category'] in ['HIGH', 'ULTRA_HIGH']]),
            "complex_decision_makers": len([d for d in decision_analysis if d['decision_level'] == 'COMPLEX']),
            "layered_distribution": {layer: len(nodes) for layer, nodes in architecture_layers.items()},
            "longest_chain_length": len(longest_chain.get('chain', [])) if longest_chain else 0
        }
    }

def categorize_frequency(hz: float) -> str:
    """Categorize node frequency based on Hz value."""
    if hz >= 100:
        return "ULTRA_HIGH"
    elif hz >= 50:
        return "HIGH"
    elif hz >= 20:
        return "MEDIUM"
    elif hz > 0:
        return "LOW"
    else:
        return "UNKNOWN"

def determine_criticality(hz: float) -> str:
    """Determine timing criticality based on frequency."""
    if hz >= 100:
        return "HARDWARE_CRITICAL"
    elif hz >= 50:
        return "REAL_TIME_CRITICAL"
    elif hz >= 20:
        return "SOFT_REAL_TIME"
    elif hz > 0:
        return "STANDARD"
    else:
        return "NON_CRITICAL"

def calculate_publication_rate(node: Dict) -> Dict:
    """Calculate publication rate and pattern."""
    pubs = node.get('pubs', [])
    hz = node.get('hz', 0)
    
    topic_patterns = {
        "sensor_topics": len([t for t in pubs if any(keyword in t.lower() for keyword in 
                                                     ['sensor', 'camera', 'lidar', 'imu', 'laser'])]),
        "control_topics": len([t for t in pubs if any(keyword in t.lower() for keyword in 
                                                     ['cmd', 'command', 'control', 'velocity'])]),
        "status_topics": len([t for t in pubs if any(keyword in t.lower() for keyword in 
                                                    ['status', 'state', 'health', 'diagnostic'])]),
        "planning_topics": len([t for t in pubs if any(keyword in t.lower() for keyword in 
                                                      ['plan', 'trajectory', 'path', 'goal'])]),
        "custom_topics": len([t for t in pubs if not any(keyword in t.lower() for keyword in 
                                                        ['sensor', 'camera', 'lidar', 'imu', 'laser',
                                                         'cmd', 'command', 'control', 'velocity',
                                                         'status', 'state', 'health', 'diagnostic',
                                                         'plan', 'trajectory', 'path', 'goal'])])
    }
    
    return {
        "total_publications": len(pubs),
        "estimated_publications_per_second": hz * len(pubs) if hz > 0 else 0,
        "topic_patterns": topic_patterns,
        "publication_pattern": "PERIODIC" if hz > 0 else "EVENT_DRIVEN"
    }

def analyze_decision_complexity(node: Dict) -> Dict:
    """Analyze decision-making complexity of a node."""
    input_count = len(node.get('subs', []))
    output_count = len(node.get('pubs', []))
    service_count = len(node.get('srvs', []))
    action_count = len(node.get('acts', []))
    
    # Calculate decision score
    score = 0
    
    # Input diversity adds complexity
    if input_count >= 5:
        score += 3
    elif input_count >= 3:
        score += 2
    elif input_count >= 2:
        score += 1
    
    # Services indicate decision-making capability
    if service_count > 0:
        score += service_count * 1.5
    
    # Actions indicate high-level decision-making
    if action_count > 0:
        score += action_count * 3
    
    # Multiple outputs indicate decision branching
    if output_count >= 3:
        score += 2
    elif output_count >= 2:
        score += 1
    
    # Determine decision level
    if score >= 7:
        level = "COMPLEX_DECISION_MAKING"
    elif score >= 4:
        level = "MODERATE_DECISION_MAKING"
    elif score >= 2:
        level = "BASIC_DECISION_MAKING"
    else:
        level = "SIMPLE_PROCESSING"
    
    return {"score": score, "level": level}

def calculate_cognitive_load(node: Dict) -> float:
    """Calculate cognitive load based on connections and responsibilities."""
    input_count = len(node.get('subs', []))
    output_count = len(node.get('pubs', []))
    service_count = len(node.get('srvs', []))
    action_count = len(node.get('acts', []))
    
    # Weighted calculation
    cognitive_load = (
        input_count * 1.0 +      # Processing inputs
        output_count * 0.8 +     # Managing outputs
        service_count * 1.5 +    # Service handling
        action_count * 2.0       # Action handling (most complex)
    )
    
    return round(cognitive_load, 2)

def analyze_communication_chains(nodes: List[Dict]) -> List[Dict]:
    """Analyze communication chains in the system."""
    # Build adjacency matrix
    adjacency = defaultdict(list)
    topic_map = defaultdict(list)
    
    # First pass: build topic to publisher mapping
    for node in nodes:
        node_name = node.get('name', '')
        for topic in node.get('pubs', []):
            topic_map[topic].append(("pub", node_name))
        for topic in node.get('subs', []):
            topic_map[topic].append(("sub", node_name))
    
    # Second pass: build adjacency based on topics
    for topic, participants in topic_map.items():
        publishers = [p[1] for p in participants if p[0] == "pub"]
        subscribers = [s[1] for s in participants if s[0] == "sub"]
        
        for pub in publishers:
            for sub in subscribers:
                if pub != sub:
                    adjacency[pub].append({
                        "target": sub,
                        "topic": topic,
                        "type": "topic",
                        "direction": "pub_to_sub"
                    })
    
    # Find all chains using BFS
    all_chains = []
    visited_nodes = set()
    
    for start_node in adjacency.keys():
        if start_node in visited_nodes:
            continue
            
        queue = [(start_node, [start_node], [])]
        
        while queue:
            current_node, current_chain, current_topics = queue.pop(0)
            visited_nodes.add(current_node)
            
            # Record chain if it has at least 2 nodes
            if len(current_chain) >= 2:
                all_chains.append({
                    "chain": current_chain.copy(),
                    "topics": current_topics.copy(),
                    "length": len(current_chain)
                })
            
            # Explore neighbors
            for neighbor in adjacency.get(current_node, []):
                if neighbor["target"] not in current_chain:  # Avoid cycles
                    new_chain = current_chain + [neighbor["target"]]
                    new_topics = current_topics + [{
                        "from": current_node,
                        "to": neighbor["target"],
                        "topic": neighbor["topic"],
                        "type": neighbor["type"]
                    }]
                    queue.append((neighbor["target"], new_chain, new_topics))
    
    # Sort by chain length
    all_chains.sort(key=lambda x: x["length"], reverse=True)
    
    return all_chains[:10]  # Return top 10 longest chains

def find_longest_communication_chain(nodes: List[Dict]) -> Optional[Dict]:
    """Find the longest communication chain in the system."""
    chains = analyze_communication_chains(nodes)
    return chains[0] if chains else None

# ===== EXISTING FUNCTIONS (updated to include professional analysis) =====

def get_ros_package_info(start_path):
    """Identifies ROS2 packages and their type (Python or C++)."""
    packages = {}
    for root, dirs, files in os.walk(start_path):
        if 'package.xml' in files:
            pkg_path = Path(root)
            pkg_name = pkg_path.name
            
            # Read package name from package.xml
            try:
                tree = ET.parse(pkg_path / 'package.xml')
                xml_root = tree.getroot()
                name_element = xml_root.find('name')
                if name_element is not None and name_element.text:
                    pkg_name = name_element.text
            except:
                pass
                
            # Determine build type
            try:
                with open(pkg_path / 'package.xml', 'r', encoding='utf-8') as f:
                    content = f.read()
                    if "ament_python" in content:
                        build_type = "python"
                    elif "ament_cmake" in content:
                        build_type = "cpp"
                    else:
                        build_type = "unknown"
            except:
                build_type = "unknown"
                
            packages[str(pkg_path)] = {
                "name": pkg_name,
                "type": build_type,
                "path": str(pkg_path)
            }
    return packages

def extract_ros_entities(file_path):
    """Uses Abstract Syntax Tree (AST) to find ROS2 entities."""
    entities = {"pubs": [], "subs": [], "srvs": [], "acts": [], "params": []}
    
    try:
        with open(file_path, "r", encoding='utf-8') as f:
            content = f.read()
            tree = ast.parse(content)

        def extract_arg_value(node, index, keyword):
            """Helps extract argument value whether positional or named"""
            for kw in node.keywords:
                if kw.arg == keyword:
                    if isinstance(kw.value, ast.Constant):
                        return kw.value.value
                    elif isinstance(kw.value, ast.Str):
                        return kw.value.s
            if len(node.args) > index:
                arg = node.args[index]
                if isinstance(arg, ast.Constant):
                    return arg.value
                elif isinstance(arg, ast.Str):
                    return arg.s
            return None

        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                if isinstance(node.func, ast.Attribute):
                    func_name = node.func.attr
                    
                    if func_name == 'create_publisher':
                        topic_name = extract_arg_value(node, index=1, keyword='topic')
                        if topic_name:
                            msg_type = extract_arg_value(node, index=0, keyword='msg_type')
                            if msg_type:
                                entities["pubs"].append(f"{msg_type}::'{topic_name}'")
                            else:
                                entities["pubs"].append(topic_name)
                    
                    elif func_name == 'create_subscription':
                        topic_name = extract_arg_value(node, index=1, keyword='topic')
                        if topic_name:
                            msg_type = extract_arg_value(node, index=0, keyword='msg_type')
                            if msg_type:
                                entities["subs"].append(f"{msg_type}::'{topic_name}'")
                            else:
                                entities["subs"].append(topic_name)
                    
                    elif func_name == 'create_service':
                        srv_name = extract_arg_value(node, index=1, keyword='srv_name')
                        if srv_name:
                            srv_type = extract_arg_value(node, index=0, keyword='srv_type')
                            if srv_type:
                                entities["srvs"].append(f"{srv_type}::'{srv_name}'")
                            else:
                                entities["srvs"].append(srv_name)
                    
                    elif func_name == 'ActionServer' or func_name == 'create_action_server':
                        action_name = extract_arg_value(node, index=2, keyword='action_name')
                        if action_name:
                            action_type = extract_arg_value(node, index=1, keyword='action_type')
                            if action_type:
                                entities["acts"].append(f"{action_type}::'{action_name}'")
                            else:
                                entities["acts"].append(action_name)
                    
                    elif func_name == 'declare_parameter':
                        param_name = extract_arg_value(node, index=0, keyword='name')
                        if param_name:
                            entities["params"].append(param_name)
                
                elif isinstance(node.func, ast.Name):
                    func_name = node.func.id
                    if func_name == 'declare_parameter':
                        param_name = extract_arg_value(node, index=0, keyword='name')
                        if param_name:
                            entities["params"].append(param_name)

    except Exception as e:
        print(f"AST error for {file_path}: {e}")
    
    return entities

def detect_cpp_entities(file_path):
    """Detection of ROS2 entities in C++ files."""
    entities = {"pubs": [], "subs": [], "srvs": [], "acts": [], "params": []}
    
    try:
        with open(file_path, "r", encoding='utf-8', errors='ignore') as f:
            content = f.read()
        
        # Publisher detection
        pub_matches = re.findall(
            r'create_publisher\s*<\s*([^>]+)\s*>\s*\(\s*["\']([^"\']+)["\']',
            content
        )
        for msg_type, topic in pub_matches:
            entities["pubs"].append(f"{msg_type.strip()}::'{topic.strip()}'")
        
        # Subscriber detection
        sub_matches = re.findall(
            r'create_subscription\s*<\s*([^>]+)\s*>\s*\(\s*["\']([^"\']+)["\']',
            content
        )
        for msg_type, topic in sub_matches:
            entities["subs"].append(f"{msg_type.strip()}::'{topic.strip()}'")
        
        # Service detection
        srv_matches = re.findall(
            r'create_service\s*<\s*([^>]+)\s*>\s*\(\s*["\']([^"\']+)["\']',
            content
        )
        for srv_type, srv_name in srv_matches:
            entities["srvs"].append(f"{srv_type.strip()}::'{srv_name.strip()}'")
        
        # Parameter detection
        param_matches = re.findall(
            r'declare_parameter\s*\(\s*["\']([^"\']+)["\']',
            content
        )
        entities["params"].extend([p.strip() for p in param_matches])
        
    except Exception as e:
        print(f"C++ error for {file_path}: {e}")
    
    return entities

class ROS2Parser:
    @staticmethod
    def build_project_tree(path: str) -> Dict:
        """Generates complete file and folder tree"""
        name = os.path.basename(path) if os.path.basename(path) else "workspace"
        
        d = {
            'name': name,
            'type': 'folder',
            'children': []
        }
        
        try:
            items = sorted(os.listdir(path))
            for item in items:
                if item in ["__pycache__", ".git", "build", "install", "log", "venv", ".vscode", ".idea"]:
                    continue
                
                full_path = os.path.join(path, item)
                
                if os.path.isdir(full_path):
                    d['children'].append(ROS2Parser.build_project_tree(full_path))
                else:
                    d['children'].append({
                        'name': item,
                        'type': 'file',
                        'size': os.path.getsize(full_path)
                    })
        except Exception as e:
            print(f"Error building tree for {path}: {e}")
        
        return d

    @staticmethod
    def find_nodes(file_path: str) -> List[Dict]:
        """Looks for Node classes and instances in main()"""
        nodes = []
        ext = os.path.splitext(file_path)[1].lower()
        
        try:
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
            
            node_names = set()
            
            # Python nodes
            if ext == '.py':
                try:
                    tree = ast.parse(content)
                    for node in ast.walk(tree):
                        if isinstance(node, ast.ClassDef):
                            for base in node.bases:
                                if isinstance(base, ast.Name) and base.id == 'Node':
                                    node_names.add(node.name)
                                elif isinstance(base, ast.Attribute) and base.attr == 'Node':
                                    node_names.add(node.name)
                except:
                    class_matches = re.findall(r"class\s+(\w+)\s*\(.*Node.*\):", content)
                    node_names.update(class_matches)
                
                try:
                    tree = ast.parse(content)
                    for node in ast.walk(tree):
                        if isinstance(node, ast.Call):
                            if isinstance(node.func, ast.Name) and node.func.id == 'Node':
                                if node.args and isinstance(node.args[0], ast.Constant):
                                    node_names.add(node.args[0].value)
                            elif isinstance(node.func, ast.Attribute):
                                if node.func.attr == 'create_node' and node.args:
                                    if isinstance(node.args[0], ast.Constant):
                                        node_names.add(node.args[0].value)
                except:
                    instance_matches = re.findall(
                        r"(?:rclpy\.create_node|Node|create_node)\(\s*['\"](.*?)['\"]", 
                        content
                    )
                    node_names.update(instance_matches)
            
            # C++ nodes
            elif ext in ['.cpp', '.hpp', '.h']:
                class_matches = re.findall(
                    r"class\s+(\w+)\s*:\s*(?:public\s+)?rclcpp::Node", 
                    content
                )
                node_names.update(class_matches)
                
                instance_matches = re.findall(
                    r"(?:std::make_shared<.*?>|rclcpp::Node::make_shared|Node)\(\s*['\"](.*?)['\"]", 
                    content
                )
                node_names.update(instance_matches)
            
            for name in node_names:
                if name and len(name.strip()) > 0:
                    package_name = "unknown"
                    package_type = "unknown"
                    
                    current_dir = os.path.dirname(file_path)
                    while current_dir and current_dir != '/':
                        if os.path.exists(os.path.join(current_dir, 'package.xml')):
                            try:
                                tree = ET.parse(os.path.join(current_dir, 'package.xml'))
                                xml_root = tree.getroot()
                                name_element = xml_root.find('name')
                                if name_element is not None and name_element.text:
                                    package_name = name_element.text
                                
                                with open(os.path.join(current_dir, 'package.xml'), 'r', encoding='utf-8') as f:
                                    xml_content = f.read()
                                    if "ament_python" in xml_content:
                                        package_type = "python"
                                    elif "ament_cmake" in xml_content:
                                        package_type = "cpp"
                                        
                                break
                            except:
                                package_name = os.path.basename(current_dir)
                                break
                        current_dir = os.path.dirname(current_dir)
                    
                    if package_name == "unknown":
                        package_name = os.path.basename(os.path.dirname(file_path))
                    
                    nodes.append({
                        "name": name.strip(),
                        "lang": "Python" if ext == '.py' else "C++",
                        "package": package_name,
                        "package_type": package_type,
                        "file": os.path.basename(file_path),
                        "file_path": file_path
                    })
                    
        except Exception as e:
            print(f"Error parsing nodes in {file_path}: {e}")
        
        return nodes

    @staticmethod
    def get_details(file_path: str) -> Dict:
        """In-depth technical analysis with function signatures"""
        res = {
            "pubs": [], "subs": [], "srvs": [], "acts": [], 
            "params": [], "hz": 0, "quality": [], "qos": []
        }
        
        try:
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
            
            # Extract Publishers with signature
            pub_matches = re.findall(r"create_publisher.*?\(\s*([^,]+)\s*,\s*['\"](.*?)['\"]", content)
            for msg_type, topic in pub_matches:
                res["pubs"].append(f"create_publisher({msg_type.strip()}, '{topic.strip()}', 10)")
            
            # Extract Subscribers with signature
            sub_matches = re.findall(r"create_subscription.*?\(\s*([^,]+)\s*,\s*['\"](.*?)['\"]", content)
            for msg_type, topic in sub_matches:
                res["subs"].append(f"create_subscription({msg_type.strip()}, '{topic.strip()}', callback)")
            
            # Extract Services
            srv_matches = re.findall(r"create_service.*?\(\s*([^,]+)\s*,\s*['\"](.*?)['\"]", content)
            for srv_type, srv_name in srv_matches:
                res["srvs"].append(f"create_service({srv_type.strip()}, '{srv_name.strip()}', callback)")
            
            # Extract Actions
            act_matches = re.findall(r"ActionServer.*?\(\s*self\s*,\s*([^,]+)\s*,\s*['\"](.*?)['\"]", content)
            for act_type, act_name in act_matches:
                res["acts"].append(f"ActionServer(self, {act_type.strip()}, '{act_name.strip()}')")
            
            # Parameters
            param_matches = re.findall(r"declare_parameter.*?\(\s*['\"](.*?)['\"]", content)
            res["params"].extend([f"declare_parameter('{p}')" for p in param_matches])
            
            # Frequency (Timer)
            timer_matches = re.findall(r"create_timer\(\s*([\d\.]+)", content)
            if timer_matches:
                res["hz"] = round(1.0 / float(timer_matches[0]), 1)
            
            # Quality
            if "time.sleep" in content: res["quality"].append("⚠️ Usage of blocking time.sleep()")
            if "while True" in content and "rclpy.spin" not in content:
                res["quality"].append("⚠️ Infinite loop without spin")
                
        except Exception as e:
            print(f"Error getting details: {e}")
        
        return res

    @staticmethod
    def parse_launch_file(launch_path: str) -> Dict:
        """Parses a Python or XML launch file"""
        result = {
            "file": os.path.basename(launch_path),
            "path": launch_path,
            "nodes": [],
            "params": [],
            "type": "unknown",
            "comments": ""
        }
        
        try:
            with open(launch_path, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
            
            # Extract comments for description
            comment_lines = re.findall(r'#\s*(.*?)$', content, re.MULTILINE)
            if comment_lines:
                result["comments"] = " ".join(comment_lines[:3])[:200]
            
            # Python launch file
            if launch_path.endswith('.py'):
                result["type"] = "python"
                node_patterns = [
                    r"Node\(\s*package=['\"](.*?)['\"].*?executable=['\"](.*?)['\"]",
                    r"Node\(\s*executable=['\"](.*?)['\"]",
                    r"LaunchConfiguration\(\s*['\"](.*?)['\"]"
                ]
                
                for pattern in node_patterns:
                    matches = re.findall(pattern, content)
                    if matches:
                        if pattern == r"LaunchConfiguration\(\s*['\"](.*?)['\"]":
                            result["params"].extend([m for m in matches if m])
                        else:
                            result["nodes"].extend([m[-1] for m in matches if m and m[-1]])
            
            # XML launch file
            elif launch_path.endswith('.xml') or launch_path.endswith('.launch'):
                result["type"] = "xml"
                node_matches = re.findall(r'<node.*?name=["\'](.*?)["\']', content)
                result["nodes"].extend([m for m in node_matches if m])
                
                param_matches = re.findall(r'<param.*?name=["\'](.*?)["\']', content)
                result["params"].extend([m for m in param_matches if m])
            
            # YAML launch file (ROS2)
            elif launch_path.endswith('.yaml') or launch_path.endswith('.yml'):
                result["type"] = "yaml"
                try:
                    with open(launch_path, 'r') as f:
                        yaml_data = yaml.safe_load(f)
                    
                    if isinstance(yaml_data, dict):
                        for key, value in yaml_data.items():
                            if isinstance(value, dict):
                                if 'executable' in value:
                                    result["nodes"].append(value.get('executable', ''))
                                if 'parameters' in value:
                                    if isinstance(value['parameters'], dict):
                                        result["params"].extend(list(value['parameters'].keys()))
                except Exception as e:
                    print(f"Error parsing YAML launch file {launch_path}: {e}")
            
            # Remove duplicates and empty strings
            result["nodes"] = list(set([n for n in result["nodes"] if n and n.strip()]))
            result["params"] = list(set([p for p in result["params"] if p and p.strip()]))
            
        except Exception as e:
            print(f"Error parsing launch file {launch_path}: {e}")
        
        return result

    @staticmethod
    def scan_launch_files(root: str) -> Dict:
        """Recursive scan for launch files organized by package"""
        packages = {}
        
        for r, _, files in os.walk(root):
            for f in files:
                lower_f = f.lower()
                if ('launch' in lower_f or 
                    f.endswith('.launch.py') or 
                    f.endswith('.launch.xml') or
                    f.endswith('_launch.py') or
                    f.endswith('.launch') or
                    ('launch' in r.lower() and (f.endswith('.py') or f.endswith('.xml') or f.endswith('.yaml') or f.endswith('.yml')))):
                    
                    launch_path = os.path.join(r, f)
                    
                    # Extract package name from path
                    package_name = "unknown"
                    path_parts = r.split(os.sep)
                    
                    for i, part in enumerate(path_parts):
                        if 'src' in part.lower() and i + 1 < len(path_parts):
                            package_name = path_parts[i + 1]
                            break
                        elif 'package' in part.lower() and i + 1 < len(path_parts):
                            package_name = path_parts[i + 1]
                            break
                    
                    if package_name == "unknown":
                        package_name = os.path.basename(os.path.dirname(r))
                        if not package_name or package_name == '.':
                            package_name = os.path.basename(r)
                    
                    # Parse launch file
                    launch_data = ROS2Parser.parse_launch_file(launch_path)
                    
                    # Add to package
                    if package_name not in packages:
                        packages[package_name] = {
                            "path": r,
                            "launch_files": [],
                            "launch_types": {"python": 0, "xml": 0, "yaml": 0, "unknown": 0}
                        }
                    
                    packages[package_name]["launch_files"].append(launch_data)
                    
                    # Update type count
                    launch_type = launch_data.get("type", "unknown")
                    if launch_type in packages[package_name]["launch_types"]:
                        packages[package_name]["launch_types"][launch_type] += 1
                    else:
                        packages[package_name]["launch_types"]["unknown"] += 1
        
        return packages

    @staticmethod
    def extract_tf_info(root: str) -> Dict:
        """Extracts TF information from files"""
        tf_data = {
            "odom": ["base_link"],
            "base_link": ["camera", "laser", "wheel_1", "wheel_2", "wheel_3", "wheel_4"],
            "custom": []
        }
        
        # Scan for URDF/XACRO files
        for r, _, files in os.walk(root):
            for f in files:
                if f.endswith(('.urdf', '.xacro')):
                    urdf_path = os.path.join(r, f)
                    try:
                        with open(urdf_path, 'r', encoding='utf-8', errors='ignore') as file:
                            urdf_content = file.read()
                        
                        # Extract links and joints
                        links = re.findall(r'<link\s+name=["\'](.*?)["\']', urdf_content)
                        joints = re.findall(r'<joint\s+name=["\'](.*?)["\'].*?<parent\s+link=["\'](.*?)["\'].*?<child\s+link=["\'](.*?)["\']', 
                                           urdf_content, re.DOTALL)
                        
                        for joint, parent, child in joints:
                            if parent and child:
                                tf_data["custom"].append([parent.strip(), child.strip()])
                        
                    except Exception as e:
                        print(f"Error parsing URDF {urdf_path}: {e}")
        
        # Scan for TF broadcaster code
        for r, _, files in os.walk(root):
            for f in files:
                if f.endswith(('.py', '.cpp')):
                    file_path = os.path.join(r, f)
                    try:
                        with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
                            content = file.read()
                        
                        # Look for TF broadcasts
                        tf_matches = re.findall(
                            r'tf_broadcaster.*?\(\s*["\'](.*?)["\'].*?["\'](.*?)["\']', 
                            content, 
                            re.DOTALL
                        )
                        
                        for parent, child in tf_matches:
                            if parent and child:
                                tf_data["custom"].append([parent.strip(), child.strip()])
                                
                    except Exception as e:
                        print(f"Error scanning TF in {file_path}: {e}")
        
        # Filter out empty custom frames
        tf_data["custom"] = [pair for pair in tf_data["custom"] if pair[0] and pair[1]]
        
        return tf_data

    @staticmethod
    def organize_nodes_by_package(nodes: List[Dict], root: str) -> Dict:
        """Organizes nodes by package"""
        nodes_by_package = defaultdict(list)
        
        for node in nodes:
            package_name = node.get('package', 'unknown')
            if package_name == 'unknown':
                file_path = node.get('file_path', '')
                if file_path:
                    # Extract package name from path
                    path_parts = file_path.split(os.sep)
                    # Look for src directory
                    for i, part in enumerate(path_parts):
                        if 'src' in part.lower() and i + 1 < len(path_parts):
                            package_name = path_parts[i + 1]
                            break
                        elif part.lower().endswith('_pkg') and i < len(path_parts):
                            package_name = part
                            break
                    
                    # If no package found, use the directory containing the file
                    if package_name == 'unknown':
                        package_name = os.path.basename(os.path.dirname(file_path))
                        if not package_name or package_name == '.':
                            package_name = os.path.basename(os.path.dirname(os.path.dirname(file_path)))
            
            nodes_by_package[package_name].append(node)
        
        return dict(nodes_by_package)

    @staticmethod
    def build_detailed_graph(nodes: List[Dict], tf_info: Dict) -> Dict[str, Any]:
        """Builds a detailed graph with optimized layouts"""
        G = nx.Graph()
        
        # Main nodes
        node_data = {}
        for node in nodes:
            node_id = f"node_{node['name'].replace(' ', '_').replace('.', '_')}"
            G.add_node(node_id, 
                      type='node',
                      name=node['name'],
                      lang=node.get('lang', 'unknown'),
                      package=node.get('package', 'unknown'))
            node_data[node_id] = node
        
        # Topics
        all_topics = set()
        for node in nodes:
            all_topics.update(node.get('pubs', []))
            all_topics.update(node.get('subs', []))
        
        for topic in all_topics:
            topic_id = f"topic_{topic.replace(' ', '_').replace('/', '_')}"
            G.add_node(topic_id, type='topic', name=topic)
        
        # Services
        all_services = set()
        for node in nodes:
            all_services.update(node.get('srvs', []))
        
        for service in all_services:
            service_id = f"service_{service.replace(' ', '_').replace('/', '_')}"
            G.add_node(service_id, type='service', name=service)
        
        # Actions
        all_actions = set()
        for node in nodes:
            all_actions.update(node.get('acts', []))
        
        for action in all_actions:
            action_id = f"action_{action.replace(' ', '_').replace('/', '_')}"
            G.add_node(action_id, type='action', name=action)
        
        # Connections
        for node in nodes:
            node_id = f"node_{node['name'].replace(' ', '_').replace('.', '_')}"
            
            # Publishers → Topics
            for topic in node.get('pubs', []):
                topic_id = f"topic_{topic.replace(' ', '_').replace('/', '_')}"
                G.add_edge(node_id, topic_id, type='publisher', 
                          color='#10b981', style='solid', thickness=2)
            
            # Topics → Subscribers
            for topic in node.get('subs', []):
                topic_id = f"topic_{topic.replace(' ', '_').replace('/', '_')}"
                G.add_edge(topic_id, node_id, type='subscriber',
                          color='#f59e0b', style='solid', thickness=2)
            
            # Services
            for service in node.get('srvs', []):
                service_id = f"service_{service.replace(' ', '_').replace('/', '_')}"
                G.add_edge(node_id, service_id, type='service',
                          color='#8b5cf6', style='dashed', thickness=1.5)
            
            # Actions
            for action in node.get('acts', []):
                action_id = f"action_{action.replace(' ', '_').replace('/', '_')}"
                G.add_edge(node_id, action_id, type='action',
                          color='#ec4899', style='dotted', thickness=1.5)
        
        # TF frames
        tf_nodes = set()
        for parent, child in tf_info.get('custom', []):
            parent_id = f"tf_{parent.replace(' ', '_').replace('-', '_')}"
            child_id = f"tf_{child.replace(' ', '_').replace('-', '_')}"
            G.add_node(parent_id, type='tf_frame', name=parent)
            G.add_node(child_id, type='tf_frame', name=child)
            G.add_edge(parent_id, child_id, type='tf_transform',
                      color='#f472b6', style='bold', thickness=3)
        
        # Organization by packages
        packages = defaultdict(list)
        for node in nodes:
            pkg = node.get('package', 'unknown')
            packages[pkg].append(node['name'])
        
        # Generate data for visualization
        graph_data = {
            'nodes': [
                {'id': n, 'type': G.nodes[n].get('type', 'unknown'), 
                 'name': G.nodes[n].get('name', n), 'package': G.nodes[n].get('package', '')}
                for n in G.nodes()
            ],
            'edges': [
                {'source': u, 'target': v, 'type': G[u][v].get('type', 'unknown'),
                 'color': G[u][v].get('color', '#94a3b8'), 
                 'style': G[u][v].get('style', 'solid')}
                for u, v in G.edges()
            ],
            'packages': dict(packages),
            'stats': {
                'total_nodes': len([n for n in G.nodes() if G.nodes[n].get('type') == 'node']),
                'total_topics': len([n for n in G.nodes() if G.nodes[n].get('type') == 'topic']),
                'total_services': len([n for n in G.nodes() if G.nodes[n].get('type') == 'service']),
                'total_actions': len([n for n in G.nodes() if G.nodes[n].get('type') == 'action']),
                'total_tf_frames': len([n for n in G.nodes() if G.nodes[n].get('type') == 'tf_frame']),
                'total_edges': len(G.edges())
            }
        }
        
        return graph_data

    @staticmethod
    def generate_mermaid_code(nodes: List[Dict], tf_info: Dict, mode: str = 'complete') -> str:
        """Generates Mermaid code for different visualization types"""
        
        if mode == 'tf_only':
            # TF view only
            code = 'graph TD\n'
            code += '    style map fill:#1e1b4b,stroke:#6366f1,color:#fff\n'
            code += '    style odom fill:#1e3a8a,stroke:#3b82f6,color:#fff\n'
            code += '    style base_link fill:#064e3b,stroke:#10b981,color:#fff\n'
            
            frames = set()
            for parent, child in tf_info.get('custom', []):
                frames.add(parent)
                frames.add(child)
            
            # Default root frame
            if 'map' in frames:
                code += '    map["map"]\n'
            if 'odom' in frames:
                code += '    odom["odom"]\n'
            if 'base_link' in frames:
                code += '    base_link["base_link"]\n'
            
            # Custom connections
            for parent, child in tf_info.get('custom', []):
                safe_parent = parent.replace(' ', '_').replace('-', '_')
                safe_child = child.replace(' ', '_').replace('-', '_')
                code += f'    {safe_parent}["{parent}"]\n'
                code += f'    {safe_child}["{child}"]\n'
                code += f'    {safe_parent} --> {safe_child}\n'
            
            return code
        
        elif mode == 'communication':
            # Communication view by package
            code = 'graph LR\n'
            
            # Group by package
            packages = defaultdict(list)
            for node in nodes:
                pkg = node.get('package', 'unknown')
                packages[pkg].append(node)
            
            # Subgraphs for each package
            for i, (pkg_name, pkg_nodes) in enumerate(packages.items()):
                if pkg_nodes:
                    code += f'  subgraph pkg_{i} ["📦 {pkg_name}"]\n'
                    for node in pkg_nodes:
                        node_id = node['name'].replace(' ', '_').replace('.', '_')
                        code += f'    {node_id}["{node["name"]}"]:::node\n'
                    code += '  end\n'
            
            # Connections
            topic_counter = 1
            for node in nodes:
                node_id = node['name'].replace(' ', '_').replace('.', '_')
                
                # Publishers
                for topic in node.get('pubs', []):
                    topic_id = f"T{topic_counter}"
                    code += f'  {node_id} -- "{topic}" --> {topic_id}(("📡")):::topic\n'
                    topic_counter += 1
                
                # Subscribers
                for topic in node.get('subs', []):
                    code += f'  T{topic_counter}(("📡")):::topic -- "{topic}" --> {node_id}\n'
                    topic_counter += 1
                
                # Services
                for service in node.get('srvs', []):
                    service_id = f"S{service[:10].replace(' ', '_')}"
                    code += f'  {node_id} -. "{service}" .-> {service_id}{{{"🛠️"}}}:::service\n'
                
                # Actions
                for action in node.get('acts', []):
                    action_id = f"A{action[:10].replace(' ', '_')}"
                    code += f'  {node_id} == "{action}" ==> {action_id}{{"⚡"}}:::action\n'
            
            code += '\n  classDef node fill:#1e1b4b,stroke:#6366f1,color:#fff\n'
            code += '  classDef topic fill:#064e3b,stroke:#10b981,color:#fff\n'
            code += '  classDef service fill:#4c1d95,stroke:#8b5cf6,color:#fff\n'
            code += '  classDef action fill:#831843,stroke:#ec4899,color:#fff\n'
            
            return code
        
        elif mode == 'simplified':
            # Simplified view (main nodes only)
            code = 'graph TD\n'
            
            for node in nodes:
                if node.get('pubs') or node.get('subs'):
                    node_id = node['name'].replace(' ', '_').replace('.', '_')
                    code += f'  {node_id}["{node["name"]}"]\n'
                    
                    # Simple connections
                    if node.get('pubs'):
                        code += f'  {node_id} -->|publishes| Topic{hash(node["name"]) % 1000}\n'
                    if node.get('subs'):
                        code += f'  Topic{hash(node["name"]) % 1000} -->|received by| {node_id}\n'
            
            return code

@app.post("/analyze")
async def analyze_zip(file: UploadFile = File(...)):
    """Main analysis endpoint"""
    sid = str(uuid.uuid4())
    temp_dir = f"temp_{sid}"
    zip_path = f"{sid}.zip"
    
    try:
        # Create temp directory
        os.makedirs(temp_dir, exist_ok=True)
        
        # Save uploaded zip
        with open(zip_path, "wb") as f:
            shutil.copyfileobj(file.file, f)
        
        # Extract zip
        with zipfile.ZipFile(zip_path, 'r') as z:
            z.extractall(temp_dir)
        
        # === STEP 1: Package identification ===
        packages_info = get_ros_package_info(temp_dir)
        
        # === STEP 2: Workspace validation ===
        validation_results = validate_workspace(temp_dir)
        
        # === STEP 3: Node analysis ===
        all_nodes = []
        package_to_nodes = defaultdict(list)
        node_tracker = set()
        
        for pkg_path, pkg_info in packages_info.items():
            pkg_name = pkg_info["name"]
            pkg_type = pkg_info["type"]
            
            print(f"Analyzing package: {pkg_name} ({pkg_type}) in {pkg_path}")
            
            # Scan source files in this package
            for r, dirs, files in os.walk(pkg_path):
                # Avoid scanning nested subpackages
                dirs_to_remove = []
                for d in dirs:
                    sub_dir = os.path.join(r, d)
                    if os.path.exists(os.path.join(sub_dir, 'package.xml')):
                        dirs_to_remove.append(d)
                
                for d in dirs_to_remove:
                    dirs.remove(d)
                
                for f in files:
                    file_path = os.path.join(r, f)
                    
                    # Analyze Python files
                    if f.endswith('.py'):
                        try:
                            found_nodes = ROS2Parser.find_nodes(file_path)
                            for node in found_nodes:
                                node_id = f"{pkg_name}_{node.get('name', 'unnamed')}_{file_path}"
                                
                                if node_id in node_tracker:
                                    continue
                                
                                node_tracker.add(node_id)
                                
                                # Extract entities via AST
                                ast_entities = extract_ros_entities(file_path)
                                
                                # Update node with AST data
                                for entity_type in ['pubs', 'subs', 'srvs', 'acts', 'params']:
                                    if not node.get(entity_type) and ast_entities[entity_type]:
                                        node[entity_type] = ast_entities[entity_type]
                                
                                # Set package information
                                node['package'] = pkg_name
                                node['package_type'] = pkg_type
                                node['package_path'] = pkg_path
                                node['source_file'] = file_path
                                
                                # Add details if missing
                                if not any(key in node for key in ['pubs', 'subs', 'srvs', 'acts']):
                                    details = ROS2Parser.get_details(file_path)
                                    node.update(details)
                                
                                # Add to collections
                                all_nodes.append(node)
                                package_to_nodes[pkg_name].append(node)
                                
                        except Exception as e:
                            print(f"Python analysis error {file_path}: {e}")
                    
                    # Analyze C++ files
                    elif f.endswith(('.cpp', '.hpp', '.h')):
                        try:
                            found_nodes = ROS2Parser.find_nodes(file_path)
                            cpp_entities = detect_cpp_entities(file_path)
                            
                            for node in found_nodes:
                                node_id = f"{pkg_name}_{node.get('name', 'unnamed')}_{file_path}"
                                
                                if node_id in node_tracker:
                                    continue
                                
                                node_tracker.add(node_id)
                                
                                # Update with C++ entities
                                for entity_type in ['pubs', 'subs', 'srvs', 'params']:
                                    if not node.get(entity_type) and cpp_entities[entity_type]:
                                        node[entity_type] = cpp_entities[entity_type]
                                
                                # Set package information
                                node['package'] = pkg_name
                                node['package_type'] = pkg_type
                                node['package_path'] = pkg_path
                                node['source_file'] = file_path
                                
                                if not any(key in node for key in ['pubs', 'subs', 'srvs']):
                                    details = ROS2Parser.get_details(file_path)
                                    node.update(details)
                                
                                # Add to collections
                                all_nodes.append(node)
                                package_to_nodes[pkg_name].append(node)
                                
                        except Exception as e:
                            print(f"C++ analysis error {file_path}: {e}")
        
        # If no packages detected, fallback to global scan
        if not packages_info:
            print("No ROS2 packages detected, doing global analysis...")
            for r, _, files in os.walk(temp_dir):
                for f in files:
                    file_path = os.path.join(r, f)
                    
                    if f.endswith(('.py', '.cpp', '.hpp', '.h')):
                        found_nodes = ROS2Parser.find_nodes(file_path)
                        for node in found_nodes:
                            node_id = f"global_{node.get('name', 'unnamed')}_{file_path}"
                            
                            if node_id in node_tracker:
                                continue
                            
                            node_tracker.add(node_id)
                            details = ROS2Parser.get_details(file_path)
                            node.update(details)
                            all_nodes.append(node)
        
        # === STEP 4: Professional behavioral analysis ===
        behavioral_analysis = perform_behavioral_analysis(all_nodes)

        # === STEP 5: Generate architecture visualization ===
        architecture_mermaid = generate_architecture_mermaid(all_nodes)
        
        # === STEP 6: Calculate metrics ===
        total_pubs = sum(len(n.get('pubs', [])) for n in all_nodes)
        total_subs = sum(len(n.get('subs', [])) for n in all_nodes)
        total_params = sum(len(n.get('params', [])) for n in all_nodes)
        
        # Convert to dict
        nodes_by_package = dict(package_to_nodes)
        
        # === STEP 7: Build packages summary ===
        packages_summary = {}
        for pkg_path, pkg_info in packages_info.items():
            pkg_name = pkg_info["name"]
            pkg_nodes = nodes_by_package.get(pkg_name, [])
            
            packages_summary[pkg_name] = {
                **pkg_info,
                'node_count': len(pkg_nodes),
                'files': list(set([n.get('source_file', '') for n in pkg_nodes if n.get('source_file')]))
            }
        
        # === STEP 8: Continue with other analyses ===
        project_tree = ROS2Parser.build_project_tree(temp_dir)
        launch_packages = ROS2Parser.scan_launch_files(temp_dir)
        tf_info = ROS2Parser.extract_tf_info(temp_dir)
        
        # Generate visualizations
        tf_mermaid = ROS2Parser.generate_mermaid_code(all_nodes, tf_info, mode='tf_only')
        comm_mermaid = ROS2Parser.generate_mermaid_code(all_nodes, tf_info, mode='communication')
        simplified_mermaid = ROS2Parser.generate_mermaid_code(all_nodes, tf_info, mode='simplified')
        detailed_graph = ROS2Parser.build_detailed_graph(all_nodes, tf_info)
        
        # === STEP 9: Prepare final response ===
        response = {
            "validation": validation_results,
            "behavioral_analysis": behavioral_analysis,
            "architecture_mermaid": architecture_mermaid,
            "packages_detected": packages_summary,
            "tree": project_tree,
            "nodes": all_nodes,
            "nodes_by_package": nodes_by_package,
            "launch_packages": launch_packages,
            "tf": tf_info,
            "visualizations": {
                "tf_mermaid": tf_mermaid,
                "communication_mermaid": comm_mermaid,
                "simplified_mermaid": simplified_mermaid,
                "detailed_graph": detailed_graph
            },
            "metrics": {
                "total_nodes": len(all_nodes),
                "total_publishers": total_pubs,
                "total_subscribers": total_subs,
                "total_parameters": total_params,
                "python_nodes": len([n for n in all_nodes if n.get('lang') == 'Python']),
                "cpp_nodes": len([n for n in all_nodes if n.get('lang') == 'C++']),
                "launch_packages_count": len(launch_packages),
                "total_launch_files": sum(len(pkg["launch_files"]) for pkg in launch_packages.values()),
                "packages_with_nodes": len(nodes_by_package),
                "ros_packages_detected": len(packages_info),
                "graph_stats": detailed_graph['stats'],
                "behavioral_summary": behavioral_analysis['summary']
            }
        }
        
        print(f"✅ Analysis complete: {len(all_nodes)} nodes analyzed")
        print(f"✅ Professional analysis summary:")
        print(f"   - Total nodes analyzed: {behavioral_analysis['summary']['total_nodes_analyzed']}")
        print(f"   - Layered distribution: {behavioral_analysis['summary']['layered_distribution']}")
        
        return response
        
    except Exception as e:
        print(f"Error analyzing workspace: {e}")
        import traceback
        traceback.print_exc()
        
        return {
            "error": str(e),
            "validation": {
                "issues": [],
                "summary": {"total_issues": 0, "errors": 0, "warnings": 0, "infos": 0, "by_type": {}, "files_affected": 0},
                "validation_passed": False
            },
            "behavioral_analysis": {},
            "architecture_mermaid": "graph TD\n    error[Analysis Error]\n",
            "packages_detected": {},
            "tree": {"name": "error", "type": "folder", "children": []},
            "nodes": [],
            "nodes_by_package": {},
            "launch_packages": {},
            "tf": {"odom": [], "base_link": [], "custom": []},
            "visualizations": {
                "tf_mermaid": "graph TD\n    error[Analysis Error]\n",
                "communication_mermaid": "graph LR\n    error[Analysis Error]\n",
                "simplified_mermaid": "graph TD\n    error[Analysis Error]\n",
                "detailed_graph": {"nodes": [], "edges": [], "packages": {}, "stats": {}}
            },
            "metrics": {}
        }
    
    finally:
        # Cleanup
        try:
            shutil.rmtree(temp_dir, ignore_errors=True)
        except:
            pass
        try:
            os.remove(zip_path)
        except:
            pass

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "ROS2 Professional Auditor"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)