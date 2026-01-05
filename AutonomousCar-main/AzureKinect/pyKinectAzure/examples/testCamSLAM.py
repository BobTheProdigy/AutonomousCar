from CamSLAM import SLAM



# add a return to eval the results

# def test_static_scene():
#     intr = load_intrinsics()
#     slam = SLAM(intr)

#     poses = []
#     for rgbd in load_dataset("static_scene"):
#         pose = slam.process_frame(rgbd)
#         poses.append(pose)

#     translations = [np.linalg.norm(p[:3, 3]) for p in poses]
#     assert max(translations) < 0.01
