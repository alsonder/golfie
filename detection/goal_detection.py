import os
from inference_sdk import InferenceHTTPClient
import base64
from PIL import Image, ImageDraw

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="EQrKZhgL8SxzifXnmxn9"
)

script_dir = os.path.dirname(os.path.abspath(__file__))
image_path = os.path.join(script_dir, "roboflow/image_1.png")

# Encode base64 (needed for infer later)
with open(image_path, "rb") as image_file:
    image_data = image_file.read()
base64_image = base64.b64encode(image_data).decode("utf-8")

result = CLIENT.infer(base64_image, model_id="golfie/5")

# Load image using PIL
image = Image.open(image_path)
draw = ImageDraw.Draw(image)

predictions = result.get("predictions", [])

for idx, prediction in enumerate(predictions, 1):
    if prediction['class'] == "smallGoal":
        print(f"Prediction {idx}:")
        print(f"  Class: {prediction['class']}")
        print(f"  Confidence: {prediction['confidence']:.2f}")
        print(f"  Bounding Box: ({prediction['x']}, {prediction['y']}) - ({prediction['x'] + prediction['width']}, {prediction['y'] + prediction['height']})")

    # Calculate top-left and bottom-right coordinates of the bounding box with a 5-pixel shift
    shifter = 12
    left = prediction['x'] - shifter
    top = prediction['y'] 
    right = prediction['x'] + prediction['width'] - shifter
    bottom = prediction['y'] + prediction['height']

    # Draw bounding box on image
    draw.rectangle([left, top, right, bottom], outline="red")

print(f"Total predictions: {len(predictions)}")

# Show or save the image with bounding box
image.show()
# image.save("output_image_with_bbox.png")
