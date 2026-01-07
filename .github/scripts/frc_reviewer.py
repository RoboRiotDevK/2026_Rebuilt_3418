import os
import sys
import time
from google import genai

# ================= CONFIGURATION =================
# Change these settings to tweak the bot's behavior
MODEL_ID = 'gemini-3-flash-preview'

SYSTEM_PROMPT = (
    "You are a Lead Software Mentor for a FIRST Robotics Competition (FRC) team using WPILib. "
    "Your goal is to catch critical issues before code is merged, while being encouraging to students.\n\n"
    
    "REVIEW CRITERIA:\n"
    "1. SAFETY: Look for undriven motors, lack of safety limits, or code that could cause physical damage.\n"
    "2. TIMING: Flag any 'Thread.sleep()', 'while' loops without exit conditions, or heavy computation inside periodic() methods.\n"
    "3. COMMAND-BASED: Ensure Subsystems are not calling other Subsystems directly. Check for proper Command lifecycle.\n"
    "4. CLEANLINESS: Flag unused variables or excessive 'System.out.println' spam.\n\n"
    
    "OUTPUT RULES:\n"
    "- If the code is solid, just say: '✅ **Code looks good! Ready to test on the robot.**'\n"
    "- If you find issues, list them as bullet points with the filename and line number (if possible).\n"
    "- Be concise. Do not explain basic Java concepts unless necessary.\n"
    "- Use Markdown for formatting.\n"
    "- **IMPORTANT:** At the very end of your response, provide a 'Per-File Summary' table. "
    "List every file analyzed, its status (✅/⚠️/❌), and a 1-sentence action item (or 'No changes needed')."
)
# =================================================

# 1. Get the Key from GitHub Secrets
API_KEY = os.environ.get("GEMINI_API_KEY")

# --- HELPER: Handle Errors Gracefully ---
def post_error_comment(error_message):
    friendly_msg = f"""
### ⚠️ AI Analysis Failed

I ran into an issue connecting to Google Gemini. 

**Reason:** `{error_message}`

**What you should do:**
1. **Don't Panic.** This is likely an issue with the Google API limit.
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

# 2. Configure the Client
if not API_KEY:
    post_error_comment("API Key is missing from GitHub Secrets.")

client = genai.Client(api_key=API_KEY)

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

    # Combine the Config Prompt with the Code
    full_prompt = SYSTEM_PROMPT + "\n\n" + full_code

    # 4. Retry Logic
    max_retries = 3
    attempt = 0
    
    while attempt < max_retries:
        try:
            print(f"🚀 Sending to Gemini (Attempt {attempt + 1}/{max_retries})...")
            
            # API Call
            response = client.models.generate_content(
                model=MODEL_ID,
                contents=full_prompt
            )

            # --- PRINT BILLING RECEIPT ---
            try:
                usage = response.usage_metadata
                in_tokens = usage.prompt_token_count
                out_tokens = usage.candidates_token_count
                total = usage.total_token_count
                
                print("\n" + "="*10 + " BILLING ESTIMATE " + "="*10)
                print(f"📥 Input Tokens (Read):  {in_tokens}")
                print(f"📤 Output Tokens (Wrote): {out_tokens}")
                print(f"💰 Total Tokens:          {total}")
                print("="*38 + "\n")
            except:
                print("Could not retrieve token usage data.")
            # -----------------------------
            
            # SUCCESS
            with open("code_review.md", "w", encoding="utf-8") as f:
                f.write(response.text)
            print("✅ Analysis complete. Saved to file.")
            return

        except Exception as e:
            error_str = str(e)
            if "429" in error_str or "ResourceExhausted" in error_str:
                print("⚠️ Rate Limit Hit (429). Cooling down for 60s...")
                time.sleep(60)
                attempt += 1
            else:
                post_error_comment(error_str)
                return

    post_error_comment("Daily API Rate Limit Exceeded (Quits after 3 tries).")

if __name__ == "__main__":
    analyze_code()
