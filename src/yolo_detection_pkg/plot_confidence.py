#!/usr/bin/env python3

import matplotlib.pyplot as plt
import csv

times = []
confidences = []

with open("confidence_log.csv", "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        times.append(float(row["Time"]))
        confidences.append(float(row["Confidence"]))

plt.figure(figsize=(10, 5))
plt.plot(times, confidences, marker='o', linestyle='-', alpha=0.7)
plt.xlabel("Time (s)")
plt.ylabel("YOLOv8 Confidence")
plt.title("YOLOv8 TurtleBot Detection Confidence vs Time")
plt.grid(True)
plt.tight_layout()
plt.show()
