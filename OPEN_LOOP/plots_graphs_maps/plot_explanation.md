
# 📍 GPS vs IMU Dead Reckoning Path Visualization

This project analyzes and visualizes real-time location data obtained from GPS and IMU sensors. It helps compare raw GPS data with IMU-based dead reckoning estimates when GPS is lost. It includes:

* A scatter plot visualization using Matplotlib
* An interactive map visualization using Folium
* Statistics on data quality, density, and frequency

---

## 🗂 Dataset Format

The input dataset (`data_interpolated.txt`) contains time-series log entries with timestamped information from both GPS and IMU sensors. Example log snippet:

```
2025-06-10 16:27:56.833,"GPS only: 12.946025, 80.211967"
2025-06-10 16:27:56.836,"Estimated (IMU only): 12.946044, 80.211964"
```

The project extracts:

* `GPS only:` positions → Blue markers
* `Estimated (GPS lost):` positions → Red markers


---

## 📊 Code Overview

### 🔹 `code 1`: Matplotlib Plot

Plots GPS-only positions vs IMU-estimated ones when GPS was unavailable.
![image](https://github.com/user-attachments/assets/3dee1e2f-6ae8-4232-b2c4-be9afd3e6370)

**Features:**

* Blue dots: Raw GPS-only locations
* Red dots: IMU estimates 
* Legend, labels, grid

**Output:** A static plot displayed inline.


---

### 🔹 `code 2`: Folium Interactive Map

Creates an HTML map (`gps_path_plot.html`) that visualizes GPS and IMU-based paths.
![image](https://github.com/user-attachments/assets/df0220af-8776-4a28-b748-12feaa15394f)


**Features:**

* Interactive zoom/pan map
* Tooltips on points
* Blue circles: GPS-only
* Red circles: IMU-estimated

**Output:** Open `gps_path_plot.html` in a browser to explore the path interactively.

---

### 🔹 `code 3`: Data Analysis & Summary

Extracts timestamps and counts GPS vs IMU waypoints to analyze:

* Total GPS-only points
* Total IMU-estimated points
* Total data points
* Average data points per second (sampling rate)

**Sample Output:**

```
Number of GPS only waypoints: 1316
Number of Estimated (GPS lost) waypoints: 618
Total number of waypoints: 1934
Average data points per second: 56.02
```

---

## 🧪 Requirements

Install dependencies via pip:

```bash
pip install matplotlib folium numpy
```

---

## 🚀 How to Run

1. Place `data_interpolated.txt` in the root directory.
2. Run each script (`code 1`, `code 2`, `code 3`) in sequence or individually.
3. Open `gps_path_plot.html` in a browser to explore the GPS path visually.

---

## 📌 Notes

* The data parsing uses regular expressions, so formats must remain consistent.
* GPS drift simulation is added for testing robustness of IMU dead reckoning.
* Timestamps are used to compute data frequency and synchronization.

---

## 📎 License

This project is for research and educational use. Please credit original sources when publishing results.
