import os
import sys
import time
import google.generativeai as genai
from google.api_core import exceptions

# 1. Get the Key from GitHub Secrets
API_KEY = os.environ.get("GEMINI_API_KEY")

# --- HELPER: Handle Errors Gracefully ---
def post_error_comment(error_message):
    """
    Writes a friendly error message to the markdown file so the
    GitHub Action will post it as a comment instead of just crashing.
    """
    friendly_msg = f"""
### ⚠️ AI Analysis Failed

I ran into an issue connecting to Google Gemini. 

**Reason:** `{error_message}`

**What you should do:**
1. **Don't Panic.** This is likely an issue with the Google API limit, not your code.
2. You can try pushing another empty commit to trigger a retry.
3. Or, ask a human mentor to review your code manually.

*(This does NOT mean your code is perfect. I just couldn't check it.)*
"""
    try:
        with open("code_review.md", "w", encoding="utf-8") as f:
            f.write(friendly_msg)
        print("✅ Error message written to file for commenting.")
    except Exception as e:
        print(f"Failed to write error file: {e}")
    
    # Exit with 0 (Success) so the Action posts the comment
    sys.exit(0)

# 2. Configure the Model
if not API_KEY:
    post_error_comment("API Key is missing from GitHub Secrets.")

genai.configure(api_key=API_KEY)
MODEL_ID = 'gemini-3-pro-preview'
model = genai.GenerativeModel(MODEL_ID)

def get_all_java_files(root_dir="."):
    java_files = []
    for root, _, files in os.walk(root_dir):
        for file in files:
            if file.endswith(".java"):
                # Exclude vendor libraries to save tokens
                if "VendorDeps" in root or "build" in root: continue
                java_files.append(os.path.join(root, file))
    return java_files

def analyze_code():
    print(f"🔍 Searching for FRC Robot Code ({MODEL_ID})...")
    files = get_all_java_files()
    
    if not files:
        post_error_comment("No .java files were found in this repository.")
        return

    # 3. Build the 'Super String'
    full_code = ""
    for file_path in files:
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                full_code += f"\n========== FILE: {file_path} ==========\n{f.read()}"
        except: pass

    print(f"✅ Found {len(files)} files. Payload ready.")

    system_prompt = (
        "You are an expert FRC Mentor and Software Engineer. "
        "Review the following FRC robot code. "
        "Focus on: Logic errors, loop timing issues, Command-based structure violations, and code cleanliness. "
        "Be concise. If the code looks good, say so. "
        "Format your response in Markdown."
    )

    # 4. Retry Logic (for Rate Limits)
    max_retries = 3
    attempt = 0
    
    while attempt < max_retries:
        try:
            print(f"🚀 Sending to Gemini (Attempt {attempt + 1}/{max_retries})...")
            response = model.generate_content(system_prompt + "\n\n" + full_code)
            
            # SUCCESS
            with open("code_review.md", "w", encoding="utf-8") as f:
                f.write(response.text)
            print("✅ Analysis complete. Saved to file.")
            return

        except exceptions.ResourceExhausted:
            print("⚠️ Rate Limit Hit (429). Cooling down for 60s...")
            time.sleep(60)
            attempt += 1

        except Exception as e:
            post_error_comment(str(e))
            return

    # If we exit the loop, we ran out of retries
    post_error_comment("Daily API Rate Limit Exceeded (Quits after 3 tries).")

if __name__ == "__main__":
    analyze_code()
