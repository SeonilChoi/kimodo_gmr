# Kimodo & GMR

## Create motion data.
```bash
kimodo_gen "A person walking stealthily." --model Kimodo-SMPLX-RP-v1 --duration 6 --output C:\kimodo\gmr\motion_data 
```

## Retargeting motion data.
```bash
python scripts\smplx_to_robot.py --smplx_file smplx\motion_data_amass.npz --robot unitree_g1 --save_path outputs\motion_data.pkl --rate_limit  
```

## Visualizing motion data.
```bash
python scripts\vis_robot_motion.py --robot unitree_g1 --robot_motion_path outputs\motion_data.pkl 
```