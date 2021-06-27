import sys

sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2

sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import sys
import numpy as np
import glob


def calibration_photo(photo_path, camera_name):
	# 设置标定板的参数 #
	size = 1.0  # 单位厘米
	x_nums = 7  # x方向上的角点个数
	y_nums = 6  # y方向上的角点个数

	# 设置(生成)标定图在世界坐标中的坐标 #
	world_point = np.zeros((x_nums * y_nums, 3), np.float32)  # 生成x_nums*y_nums个坐标，每个坐标包含x,y,z三个元素
	world_point[:, :2] = np.mgrid[:x_nums, :y_nums].T.reshape(-1, 2)  # mgrid[]生成包含两个二维矩阵的矩阵，每个矩阵都有x_nums列,y_nums行 .T矩阵的转置 reshape()重新规划矩阵，但不改变矩阵元素
	world_point = world_point * size

	# 保存角点坐标
	world_position = []
	image_position = []

	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # 设置角点查找限制
	images = glob.glob(photo_path)  # 获取所有标定图

	for image_path in images:
		image = cv2.imread(image_path)
		gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

		ok, corners = cv2.findChessboardCorners(gray, (x_nums, y_nums), None)  # 查找角点

		if ok:
			world_position.append(world_point)  # 把每一幅图像的世界坐标放到world_position中
			exact_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)  # 获取更精确的角点位置
			image_position.append(exact_corners)  # 把获取的角点坐标放到image_position中
			# 可视化角点 #
			# image = cv2.drawChessboardCorners(image,(x_nums,y_nums),exact_corners,ok)
			# cv2.imshow('image_corner',image)
			# cv2.waitKey(5000)

	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(world_position, image_position, (1280, 720), None, None)  # 计算内参

	np.savez("./" +camera_name + "intrinsic_parameters.npz", mtx=mtx, dist=dist)  # 将内参保存起来

	# 打印内参 #
	print("*********************** intrinsic parameters of ", camera_name, "***********************")
	np.set_printoptions(suppress=True)
	print("mtx:\n", mtx, "\n")
	print("dist:\n", dist, "\n")

	# 计算偏差 #
	mean_error = 0
	for i in range(len(world_position)):
		image_position2, _ = cv2.projectPoints(world_position[i], rvecs[i], tvecs[i], mtx, dist)
		error = cv2.norm(image_position[i], image_position2, cv2.NORM_L2) / len(image_position2)
		mean_error += error
	print("total error: ", mean_error / len(image_position))


RGB_photo_path = "../save_checkboard_img/RGB/*"  # RGB图像保存路径
IR_photo_path = "../save_checkboard_img/IR/*"  # IR图像保存路径
if __name__ == "__main__":
	calibration_photo(RGB_photo_path, "RGB_camera")
	calibration_photo(IR_photo_path, "IR_camera")