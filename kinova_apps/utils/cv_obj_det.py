import cv2
import numpy as np
from pydantic import BaseModel
from enum import StrEnum
from shapely.geometry import Polygon, mapping
from shapely.geometry.base import BaseGeometry


class OBJECT_CLASS(StrEnum):
    CUBE = "cube"
    BALL = "ball"


class DetObject(BaseModel):
    geom: BaseGeometry
    obj_class: OBJECT_CLASS

    class Config:
        arbitrary_types_allowed = True
        json_encoders = {
            BaseGeometry: mapping
        }


class ObjectDetector:
    def __init__(self, color_ranges: dict):
        self.color_ranges = color_ranges

    def detect(self, image: np.ndarray) -> dict[str, list[DetObject]]:
        
        image = cv2.GaussianBlur(image, (5, 5), 0)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        d = {}
        for color, bounds in self.color_ranges.items():
            lower = bounds['lower']
            upper = bounds['upper']
            lower_np = np.array(lower, dtype=np.uint8)
            upper_np = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(hsv_image, lower_np, upper_np)
            # erode and dilate to remove noise
            # mask = cv2.erode(mask, None, iterations=2)
            # mask = cv2.dilate(mask, None, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours is None or len(contours) == 0:
                continue

            filteerd_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 1000]

            detected_objects = []

            for cnt in filteerd_contours:
                hull = cv2.convexHull(cnt)
                epsilon = 0.02 * cv2.arcLength(hull, True)
                approx = cv2.approxPolyDP(hull, epsilon, True)
                
                area = cv2.contourArea(approx)
                perimeter = cv2.arcLength(approx, True)
                if perimeter == 0: continue
                circularity = 4 * np.pi * (area / (perimeter * perimeter))

                if circularity > 0.9 and len(approx) > 5:
                    detected_objects.append(DetObject(geom=Polygon(approx.reshape(-1, 2)), obj_class=OBJECT_CLASS.BALL))
                else:
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    if 0.8 < aspect_ratio < 1.2 and len(approx) > 3:
                        detected_objects.append(DetObject(geom=Polygon(approx.reshape(-1, 2)), obj_class=OBJECT_CLASS.CUBE))


            d[color] = detected_objects

        return d

    def draw_detections(self, image, detections):
        for color, objs in detections.items():
            for obj in objs:
                poly = obj.geom
                pts = np.array(poly.exterior.coords, np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(image, [pts], isClosed=True, color=(255, 0, 0), thickness=2)
                M = cv2.moments(pts)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)
                    cv2.putText(image, f"{color} {obj.obj_class}", (cX - 20, cY - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        return image


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Object Detection")
    parser.add_argument("-i", "--image", type=str, help="Path to the image file")
    parser.add_argument("-hsv", "--hsv_config", type=str, help="Path to the HSV config YAML file")
    args = parser.parse_args()

    img_path = args.image
    hsv_colors_yaml = args.hsv_config

    assert img_path is not None, "Please provide an image path using --image"
    assert hsv_colors_yaml is not None, "Please provide an HSV config path using --hsv_config"
    
    import yaml
    with open(hsv_colors_yaml, 'r') as file:
        color_ranges = yaml.safe_load(file)

    detector = ObjectDetector(color_ranges=color_ranges)

    image = cv2.imread(img_path)
    detections = detector.detect_all(image)
    output_image = detector.draw_detections(image.copy(), detections)

    cv2.namedWindow("Detected Objects", cv2.WINDOW_NORMAL)
    cv2.imshow("Detected Objects", output_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
