from PIL import Image
from google import genai
import os
import json
import re

GEMINI_API_KEY = "AIzaSyAfUI8Z-DQqSD196GfD5Hp54ZnVhfbn9C8"
client = genai.Client(api_key=GEMINI_API_KEY)


with open("../audio-response/command.json", "r") as file:
    object_name = json.loads(file.read())["commands"][0]["params"]["object"]
# print(object_name)
# object_name = "Green Bottle"
image_folder = "../image-feed"

output_file = "../image-response/response.json"

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

def is_valid_hsv(hsv):
    return (
        isinstance(hsv, list)
        and len(hsv) == 3
        and all(isinstance(x, int) and 0 <= x <= 255 for x in hsv)
    )

def is_valid_response(resp):
    """
    Validates Gemini response with all required fields and formats.
    """
    if not isinstance(resp, dict):
        return False
    if resp.get("is_object") not in ["Yes", "No"]:
        return False
    if not is_valid_hsv(resp.get("lower_hsv", [])):
        return False
    if not is_valid_hsv(resp.get("upper_hsv", [])):
        return False
    return True

def caption_all_images(model_name="gemini-2.0-flash"):
    all_responses = {}

    for filename in os.listdir(image_folder):
        if not filename.lower().endswith((".png", ".jpg", ".jpeg")):
            continue

        image_path = os.path.join(image_folder, filename)

        try:
            image = Image.open(image_path)

            prompt = f"""You're an object detection model. Your goal is to:
- Respond if you see the mentioned object in the given picture. You should only respond with a 'Yes' or a 'No'.
- If you see the mentioned object in the picture, return the lower and upper hsvs of the color of the object such that a segmentation mask can be effectively generated for it.
- If no object is detected in the frame then return [0, 0, 0] for lower_hsv and [0, 0, 0] for upper_hsv.

Return your answer strictly in the following JSON format:
{{
  "is_object": "Yes" or "No",
  "lower_hsv": [H, S, V],
  "upper_hsv": [H, S, V]
}}

The object to find is: {object_name}"""

            response = client.models.generate_content(
                model=model_name,
                contents=[image, prompt]
            )

            raw_text = response.text.strip()
            parsed = extract_json_block(raw_text)

            if not is_valid_response(parsed):
                print(f"Invalid response format for {filename}:\n{raw_text}")
                parsed = {
                    "is_object": "Error",
                    "lower_hsv": [0, 0, 0],
                    "upper_hsv": [0, 0, 0]
                }

            all_responses[filename] = parsed
            print(f"{filename}: {parsed}")

        except Exception as e:
            print(f"Error processing {filename}: {e}")
            all_responses[filename] = {
                "is_object": "Error",
                "lower_hsv": [0, 0, 0],
                "upper_hsv": [0, 0, 0]
            }

    with open(output_file, "w") as out_file:
        json.dump(all_responses, out_file, indent=2)
        print(f"Saved full response to {output_file}")

if __name__ == "__main__":
    caption_all_images()
