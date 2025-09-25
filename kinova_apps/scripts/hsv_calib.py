#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import tkinter as tk
from tkinter import filedialog
from tkinter import ttk
from PIL import Image, ImageTk
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class CalibrateHSV(Node):
    def __init__(self) -> None:
        super().__init__("calibrate_hsv")

        # declare image and yaml files are parameters
        self.declare_parameter("image_path", "")
        image_path = self.get_parameter("image_path").get_parameter_value().string_value
        if not image_path or not os.path.isfile(image_path):
            raise FileNotFoundError(f"Image file not found: {image_path}")

        # get yaml file from package share path
        share_path = get_package_share_directory("kinova_apps")
        default_yaml = os.path.join(share_path, "config", "hsv_colors.yaml")

        self.declare_parameter("hsv_color_config", default_yaml)
        self.hsv_color_yaml = self.get_parameter("hsv_color_config").get_parameter_value().string_value
        if self.hsv_color_yaml and os.path.isfile(self.hsv_color_yaml):
            with open(self.hsv_color_yaml, "r") as f:
                self.get_logger().info(f"Loading HSV config from {self.hsv_color_yaml}")
                self.hsv_color_configs = yaml.safe_load(f)

        self.img = cv2.imread(image_path)
        if self.img is None:
            raise FileNotFoundError(f"Image not found: {image_path}")

        # Tkinter root
        self.root = tk.Tk()
        self.root.title("HSV Calibration")

        # Mode selector
        self.mode = tk.StringVar(value="manual")
        self._build_mode()

        # Manual sliders
        self.h_min = tk.IntVar(value=0)
        self.s_min = tk.IntVar(value=0)
        self.v_min = tk.IntVar(value=0)
        self.h_max = tk.IntVar(value=179)
        self.s_max = tk.IntVar(value=255)
        self.v_max = tk.IntVar(value=255)
        self._build_sliders()

        # Predefined colors (checkbox + editable entries)
        self.color_vars = {}
        self.color_entries = {}
        self._build_color_table()

        # Preview image
        self.preview_label = ttk.Label(self.root)
        self.preview_label.pack(padx=10, pady=10)

        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(pady=5)
        ttk.Button(btn_frame, text="Save Config", command=self.save_config).pack(side="left", padx=5)

    def _build_mode(self):
        frame = ttk.LabelFrame(self.root, text="Mode")
        frame.pack(padx=10, pady=5, fill="x")
        ttk.Radiobutton(frame, text="Manual", variable=self.mode, value="manual").pack(side="left", padx=6)
        ttk.Radiobutton(frame, text="Color", variable=self.mode, value="color").pack(side="left", padx=6)

    def _build_sliders(self):
        frame = ttk.LabelFrame(self.root, text="Manual HSV Range")
        frame.pack(padx=10, pady=5, fill="x")

        def make_slider(label, var, max_val):
            row = ttk.Frame(frame)
            row.pack(fill="x", padx=5, pady=2)
            ttk.Label(row, text=label, width=8).pack(side="left")

            # Scale returns float → round and set IntVar
            def on_slide(val, v=var):
                v.set(int(float(val)))

            scale = ttk.Scale(
                row,
                from_=0,
                to=max_val,
                variable=var,
                orient="horizontal",
                command=on_slide,   # ensure IntVar stays int
            )
            scale.pack(side="left", expand=True, fill="x")
            ttk.Label(row, textvariable=var, width=4).pack(side="right")

        make_slider("H Min", self.h_min, 179)
        make_slider("S Min", self.s_min, 255)
        make_slider("V Min", self.v_min, 255)
        make_slider("H Max", self.h_max, 179)
        make_slider("S Max", self.s_max, 255)
        make_slider("V Max", self.v_max, 255)

    def _build_color_table(self):
        frame = ttk.LabelFrame(self.root, text="Predefined Colors")
        frame.pack(padx=10, pady=5, fill="x")

        header = ttk.Frame(frame)
        header.pack(fill="x", padx=5)
        ttk.Label(header, text="Color", width=10).pack(side="left")
        ttk.Label(header, text="Lower (H,S,V)", width=18).pack(side="left")
        ttk.Label(header, text="Upper (H,S,V)", width=18).pack(side="left")

        for color, ranges in self.hsv_color_configs.items():
            row = ttk.Frame(frame)
            row.pack(fill="x", padx=5, pady=1)

            var = tk.BooleanVar(value=False)
            self.color_vars[color] = var

            def make_callback(c=color):
                return lambda: self._apply_color_to_sliders(c) if self.color_vars[c].get() else None

            ttk.Checkbutton(row, text=color, variable=var, width=10,
                            command=make_callback(color)).pack(side="left")

            lower_str = tk.StringVar(value=",".join(map(str, ranges["lower"])))
            upper_str = tk.StringVar(value=",".join(map(str, ranges["upper"])))
            self.color_entries[color] = (lower_str, upper_str)

            ttk.Entry(row, textvariable=lower_str, width=18).pack(side="left", padx=2)
            ttk.Entry(row, textvariable=upper_str, width=18).pack(side="left", padx=2)

    def _apply_color_to_sliders(self, color: str):
        """When a color is selected, copy its HSV values into the manual sliders."""
        lower_str, upper_str = self.color_entries[color]
        try:
            lower = np.fromstring(lower_str.get(), dtype=int, sep=",")
            upper = np.fromstring(upper_str.get(), dtype=int, sep=",")
            if lower.size == 3 and upper.size == 3:
                self.h_min.set(int(lower[0]))
                self.s_min.set(int(lower[1]))
                self.v_min.set(int(lower[2]))
                self.h_max.set(int(upper[0]))
                self.s_max.set(int(upper[1]))
                self.v_max.set(int(upper[2]))
        except ValueError:
            pass


    def process_frame(self):
        img = self.img
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if self.mode.get() == "manual":
            # Manual HSV mask
            lower = np.array([self.h_min.get(), self.s_min.get(), self.v_min.get()])
            upper = np.array([self.h_max.get(), self.s_max.get(), self.v_max.get()])
            mask = cv2.inRange(hsv, lower, upper)

        else:  # "color"
            mask = None
            for color, var in self.color_vars.items():
                if var.get():
                    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
                    lower_str, upper_str = self.color_entries[color]
                    try:
                        lower = np.fromstring(lower_str.get(), dtype=int, sep=",")
                        upper = np.fromstring(upper_str.get(), dtype=int, sep=",")
                        if lower.size == 3 and upper.size == 3:
                            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
                    except ValueError:
                        continue

        if mask is None:
            output = img
        else:
            output = cv2.bitwise_and(img, img, mask=mask)

        # Show inside Tkinter
        rgb = cv2.cvtColor(output, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(rgb)
        # Resize preview if too big
        # max_w = 800
        # if pil_img.width > max_w:
        #     scale = max_w / pil_img.width
        #     pil_img = pil_img.resize((int(pil_img.width*scale), int(pil_img.height*scale)), Image.BILINEAR)

        tk_img = ImageTk.PhotoImage(pil_img)
        self.preview_label.configure(image=tk_img)
        self.preview_label.image = tk_img

        self.root.after(30, self.process_frame)

    def save_config(self):
        """Save current HSV configuration to a JSON file"""

        new_hsv_color_configs = {}
        for color, var in self.color_vars.items():
            lower_str, upper_str = self.color_entries[color]
            try:
                lower = np.fromstring(lower_str.get(), dtype=int, sep=",")
                upper = np.fromstring(upper_str.get(), dtype=int, sep=",")
                if lower.size == 3 and upper.size == 3:
                    new_hsv_color_configs[color] = {
                        "lower": lower.tolist(),
                        "upper": upper.tolist(),
                    }
            except ValueError:
                continue

        # Ask where to save
        file_path = filedialog.asksaveasfilename(
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")],
            title="Save HSV Config"
        )
        if file_path:
            with open(file_path, "w") as f:
                yaml.dump(new_hsv_color_configs, f)
            print(f"Config saved to {file_path}")


    def run(self):
        self.root.after(30, self.process_frame)
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)

    calibrator = CalibrateHSV()
    calibrator.run()

    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

