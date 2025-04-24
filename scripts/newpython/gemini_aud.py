import google.generativeai as genai
import mimetypes
import json
import re

def load_audio(path):
    mime_type, _ = mimetypes.guess_type(path)
    if not mime_type or not mime_type.startswith("audio/"):
        raise ValueError(f"Invalid audio file: {path}")
    with open(path, "rb") as f:
        return {"mime_type": mime_type, "data": f.read()}


def extract_json_block(text):
    """
    Extracts the first valid JSON object from a Gemini response.
    """
    match = re.search(r'{.*}', text, re.DOTALL)
    if match:
        try:
            return json.loads(match.group())
        except json.JSONDecodeError:
            print("JSON decode error for block:", match.group())
            return None
    return None


def generate_audio_response(audio_path="../user_command.wav"):

    GEMINI_API_KEY = "AIzaSyAfUI8Z-DQqSD196GfD5Hp54ZnVhfbn9C8"
    genai.configure(api_key=GEMINI_API_KEY)
    model = genai.GenerativeModel("gemini-2.0-flash-lite")

    prompt = """
    You are a robot command interpreter. The user will speak a command to the jetson nano robot.
    Convert that audio command into structured JSON with the following format:

    {
    "commands": [

        { "type": "scanobj", "params": { "object": "..." } },
    ]
    }

    Only respond with valid JSON. Do not explain.
    """

    audio_blob = load_audio(audio_path)

    response = model.generate_content([prompt, audio_blob], stream=False)

    OUTPUT_COMMAND_FILE = "../audio-response/command.json"  # Replace with full path if needed

    # ==== 5. PARSE & DISPLAY ====
    try:
        response_text = response.text.strip()
        commands = extract_json_block(response_text)

        print("Parsed Robot Commands:\n")
        print(json.dumps(commands, indent=2))

        # Save to JSON file (overwrite)
        with open(OUTPUT_COMMAND_FILE, "w") as f:
            json.dump(commands, f, indent=2)
            print(f"Commands saved to {OUTPUT_COMMAND_FILE}")

    except Exception as e:
        print("Failed to parse Gemini response:")
        print(response.text)
        print("Error:", e)


generate_audio_response()