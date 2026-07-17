# GhostTrack 🚛
AI-powered blind spot detection for Indian trucks.

## Problem Statement
Indian trucks have large blind spots on both sides, causing accidents involving 
pedestrians, cyclists, and two-wheelers. GhostTrack uses AI and radar to detect 
vulnerable road users in these blind zones and alert the driver in real time.

## How It Works
1. **Cameras** mounted on left and right blind zones capture live video feed
2. **YOLOv8 model** processes each frame to detect persons, cyclists, 
   auto-rickshaws, and e-rickshaws
3. **mmWave Radar** measures the exact distance of detected objects
4. **ESP32** fuses radar and camera data and sends alerts via UART to Raspberry Pi 5
5. **Alert System** classifies threats as:
   - 🔴 CRITICAL — object very close, immediate danger -> (output- RED light on A-pillar & heavy haptic in steering)
   - 🟡 AWARENESS — object detected, driver should be cautious -> (output- YELLOW light on A-pillar & low haptic in steering)
   - 🟢 SAFE — no threat detected
6. **Driver gets alerted** in real time to avoid accidents

## Team
**S Leharin Nisha (Lead Creator & System Architect)**

Contribution: End-to-end system conceptualization, problem statement formulation, full-stack software development, web interface engineering, and training/deployment of the core machine learning models (best.pt). Developed baseline architecture independently over a 6-month timeline.

**Rajamaran (Hardware Implementation Support)**

Contribution: Assisted with physical circuit assembly, connecting the ESP32 microcontroller with the radar module, LEDs, and servo motor peripherals during the hackathon phase.
**Sumith (Hardware Implementation Support)**

Contribution: Assisted with physical circuit assembly, connecting the ESP32 microcontroller with the radar module, LEDs, and servo motor peripherals during the hackathon phase.
## Tech Stack
- YOLOv8 (COCO + custom ghosttrack_best.pt)
- Raspberry Pi 5 + ESP32
- mmWave Radar
- Google Colab + Gradio UI

## Demo
- Live Site: https://ghosttruck.onrender.com
- DrversApp:  https://ghosttrackdriveapp-git-7b2cba-leharinshainsha05-stacks-projects.vercel.app/
- Journey: https://ghosttrack-journey-git-main-leharinshainsha05-stacks-projects.vercel.app/

## Model Training
- Kaggle Notebook: https://www.kaggle.com/code/leharinnisha/ghosttrack

## License
© 2025 S Leharin. All rights reserved.  
This project is not open for public use or distribution.
