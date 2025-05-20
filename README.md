# A* Path Planning in 2D Occupancy Grid – MATLAB

## 📌 Project Overview

This project implements the **A\*** (A-star) pathfinding algorithm on a 2D **occupancy grid** to plan a collision-free path for a mobile robot from a given start to a goal location. The environment includes polygonal obstacles loaded from `.mat` files. The implementation is written in **MATLAB** and was developed for an academic robotics course.

## 🧠 Key Features

- Grid generation from polygon-based obstacles
- A\* algorithm with:
  - Optional diagonal movement
  - Multiple heuristic types: Manhattan, Chebyshev, Euclidean
- Efficient parent-matrix based backtracking
- Clear visualization of occupancy grid and final path

## 📂 Project Structure
A_STAR/ 

├── obstacle1.mat

├── obstacle2.mat

├── obstacle3.mat

├── main.m # main execution file

└── README.md

## 🚀 How to Run

1. Open the project in **MATLAB**.
2. Make sure the files `obstacle1.mat`, `obstacle2.mat`, and `obstacle3.mat` are in the same folder as `main.m`.
3. Adjust start and goal positions in `main.m` as needed:

   ```matlab
   startNode = [5.2, 15.5];
   goalNode  = [22.7, 13.1];
4. Run the script:
   main

## 📈 Sample Visual Output

A typical output plot shows the path traversing from start to goal while avoiding obstacles, with the occupancy grid displayed in grayscale.

## 🧭 Possible Extensions

* Dynamic obstacle handling
* GUI-based interactive start/goal selection
* Integration with real robot path execution
* Export of path to ROS-compatible formats

