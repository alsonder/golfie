import os
from inference_sdk import InferenceHTTPClient
import base64
from PIL import Image, ImageDraw

def detect_small_goal(image_path, shifter=12, model_id="golfie/5"):
    CLIENT = InferenceHTTPClient(
        api_url="https://detect.roboflow.com",
        api_key="EQrKZhgL8SxzifXnmxn9"
    )

    # Encode base64 (needed for infer later)
    with open(image_path, "rb") as image_file:
        image_data = image_file.read()
    base64_image = base64.b64encode(image_data).decode("utf-8")

    result = CLIENT.infer(base64_image, model_id=model_id)

    # Load image using PIL
    image = Image.open(image_path)
    draw = ImageDraw.Draw(image)

    small_goal_coordinates = []

    predictions = result.get("predictions", [])

    for idx, prediction in enumerate(predictions, 1):
        if prediction['class'] == "smallGoal":
            x = prediction['x'] - shifter
            y = prediction['y']
            small_goal_coordinates.append((x, y))
            print(f"Prediction {idx}:")
            print(f"  Class: {prediction['class']}")
            print(f"  Confidence: {prediction['confidence']:.2f}")
            print(f"  Bounding Box: ({x}, {y}) - ({x + prediction['width']}, {y + prediction['height']})")

            # Draw bounding box on image
            draw.rectangle([x, y, x + prediction['width'], y + prediction['height']], outline="red")

    print(f"Total smallGoal predictions: {len(small_goal_coordinates)}")

    # Show or save the image with bounding box
    image.show()
    # image.save("output_image_with_bbox.png")

    return small_goal_coordinates

# Example usage:
#image_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "roboflow/image_1.png")
#small_goal_coords = detect_small_goal(image_path, shifter=12)