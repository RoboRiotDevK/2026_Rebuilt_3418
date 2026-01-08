import os
import sys
import time
import re
from google import genai

# ================= CONFIGURATION =================
MODEL_ID = 'gemini-3-flash-preview'

BASE_SYSTEM_PROMPT = (
    "You are a Lead Software Mentor for a FIRST Robotics Competition (FRC) team.\n"
    "Your goal: provide specific, actionable feedback on the code provided.\n\n"
    "REVIEW RULES:\n"
    "1. SAFETY: Always flag undriven motors, lack of safety limits, or code that could cause physical damage.\n"
    "2. TIMING: Flag 'Thread.sleep()', 'while' loops without exit conditions, or heavy computation in periodic() methods.\n"
    "3. STRUCTURE: Ensure Subsystems do not call other Subsystems directly. Check Command lifecycles.\n"
    "4. FORMAT: Use Markdown. List issues with filename/line number.\n"
    "5. SUMMARY: End with a table listing the status (✅/⚠️/❌) of every analyzed file.\n"
)
# =================================================

API_KEY = os.environ.get("GEMINI_API_KEY")
CHANGED_FILES_ENV = os.environ.get("CHANGED_FILES", "")

def post_error_comment(error_message):
    friendly_msg = f"### ⚠️ AI Analysis Failed\n\nReason: `{error_message}`"
    try:
        with open("code_review.md", "w", encoding="utf-8") as f:
            f.write(friendly_msg)
    except Exception: pass
    sys.exit(0)

if not API_KEY:
    post_error_comment("API Key is missing from GitHub Secrets.")

client = genai.Client(api_key=API_KEY)

def get_all_java_files(root_dir="."):
    java_files = []
    for root, _, files in os.walk(root_dir):
        for file in files:
            if file.endswith(".java"):
                if "VendorDeps" in root or "build" in root: continue
                path = os.path.join(root, file).replace("\\", "/")
                if path.startswith("./"): path = path[2:]
                java_files.append(path)
    return java_files

def parse_java_file(file_path):
    """
    Scans a Java file to extract:
    1. The full class name (package + class)
    2. List of imported classes
    """
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
            
        pkg_match = re.search(r'package\s+([\w\.]+);', content)
        package = pkg_match.group(1) if pkg_match else ""

        class_match = re.search(r'public\s+(?:abstract\s+)?(?:final\s+)?(?:class|interface|enum)\s+(\w+)', content)
        class_name = class_match.group(1) if class_match else ""

        imports = set(re.findall(r'import\s+([\w\.]+);', content))
        
        full_name = f"{package}.{class_name}" if package and class_name else class_name

        return {
            "path": file_path,
            "full_name": full_name,
            "imports": imports,
            "content": content
        }
    except Exception:
        return None

def get_impact_graph(all_files, changed_files_list):
    """
    Returns:
    1. priority_files: Set of files to review.
    2. context_files: Set of files for reference only.
    """
    # 1. Parse EVERYTHING first to build the map
    file_map = {} 
    class_map = {} 

    for f in all_files:
        data = parse_java_file(f)
        if data:
            file_map[f] = data
            if data['full_name']:
                class_map[data['full_name']] = f

    # 2. Identify the "Seed" files (The ones that actually changed)
    seed_files = set()
    if changed_files_list:
        for changed in changed_files_list:
            clean_changed = changed.strip().replace("\\", "/")
            if clean_changed in file_map:
                seed_files.add(clean_changed)
    
    # --- CRITICAL FIX FOR SCHEDULED JOBS ---
    # If no specific changes found (e.g. Schedule run), return ALL as priority
    if not seed_files:
        return set(all_files), []

    priority_files = set(seed_files)

    # 3. Expand Blast Radius (1 level deep)
    for seed in seed_files:
        seed_data = file_map[seed]
        seed_class = seed_data['full_name']
        seed_imports = seed_data['imports']

        # A. DOWNSTREAM (What does the changed file use?)
        for imp in seed_imports:
            if imp in class_map:
                priority_files.add(class_map[imp])

        # B. UPSTREAM (Who uses the changed file?)
        if seed_class:
            for path, data in file_map.items():
                if path == seed: continue
                if seed_class in data['imports']:
                    priority_files.add(path)
                elif seed_class.split('.')[-1] in data['content']:
                    priority_files.add(path)

    # 4. Separate Context
    context_files = [f for f in all_files if f not in priority_files]
    
    return priority_files, context_files

def analyze_code():
    print(f"🔍 Searching for FRC Robot Code ({MODEL_ID})...")
    all_files = get_all_java_files()
    
    if not all_files:
        post_error_comment("No .java files were found.")
        return

    # Parse inputs from Env
    changed_list = [f for f in CHANGED_FILES_ENV.split(',') if f.strip()]
    
    print("Mapping dependencies...")
    priority_files, context_files = get_impact_graph(all_files, changed_list)
    
    # --- DETECT MODE & DYNAMIC HEADERS ---
    is_full_scan = (len(context_files) == 0)
    
    if is_full_scan:
        print(f"✅ Mode: FULL AUDIT ({len(priority_files)} files).")
        # Rename the header so the AI realizes it's looking at the whole project
        section_header = "--- 📂 PROJECT FILES (FULL AUDIT) ---"
        mode_instruction = (
            "\n\n🚨 **MODE: FULL SYSTEM AUDIT**\n"
            "You are reviewing the entire codebase for a weekly report.\n"
            "Treat ALL files below as active code to be reviewed.\n"
            "Flag ANY critical safety issues, bad practices, or logic errors found in the 'PROJECT FILES' section."
        )
    else:
        print(f"✅ Mode: PR REVIEW ({len(priority_files)} Priority, {len(context_files)} Context).")
        section_header = "--- 🔥 PRIORITY FILES (FOCUS HERE) ---"
        mode_instruction = (
            "\n\n🚨 **MODE: PULL REQUEST REVIEW**\n"
            "Focus strictly on the 'PRIORITY FILES' section (Changed files + dependencies).\n"
            "Use 'CONTEXT FILES' ONLY for reference (checking Constants, parent classes, etc).\n"
            "Do NOT report issues in Context Files unless they directly break the Priority Files."
        )

    # Build Payload
    full_prompt = BASE_SYSTEM_PROMPT + mode_instruction + "\n\n"

    # Dynamic Header Injection
    full_prompt += f"{section_header}\n"
    for path in priority_files:
        with open(path, "r", encoding="utf-8") as f:
            full_prompt += f"\n========== FILE: {path} ==========\n{f.read()}"

    # Only add Context section if it actually exists (It won't in Full Scan)
    if context_files:
        full_prompt += "\n\n--- 📖 CONTEXT FILES (REFERENCE ONLY) ---\n"
        for path in context_files:
            with open(path, "r", encoding="utf-8") as f:
                full_prompt += f"\n========== FILE: {path} ==========\n{f.read()}"

    # Retry Logic
    max_retries = 3
    for attempt in range(max_retries):
        try:
            print(f"🚀 Sending to Gemini (Attempt {attempt + 1})...")
            response = client.models.generate_content(
                model=MODEL_ID,
                contents=full_prompt
            )
            
            with open("code_review.md", "w", encoding="utf-8") as f:
                f.write(response.text)
            print("✅ Analysis complete.")
            return

        except Exception as e:
            if "429" in str(e) or "ResourceExhausted" in str(e):
                print("⚠️ Rate Limit. Waiting 60s...")
                time.sleep(60)
            else:
                post_error_comment(str(e))
                return

    post_error_comment("Rate Limit Exceeded.")

if __name__ == "__main__":
    analyze_code()
