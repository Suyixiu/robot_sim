import sys

sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2

sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import sys
import numpy as np
import glob


# 读取两个相机内参 #
RGB_mtx, RGB_dist = [], []
IR_mtx, IR_dist = [], []
with np.load("../../camera_calibration/scripts/RGB_cameraintrinsic_parameters.npz") as X:
    RGB_mtx, RGB_dist = [X[i] for i in ("mtx", "dist")]
with np.load("../../camera_calibration/scripts/IR_cameraintrinsic_parameters.npz") as X:
    IR_mtx, IR_dist = [X[i] for i in ("mtx", "dist")]


def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


# 求外参 #
def calibration_photo(photo_path, mtx, dist, windows_name):
    # 设置要标定的角点个数 #
    size = 1.0  # 单位厘米
    x_nums = 7  # x方向上的角点个数
    y_nums = 6

    # 设置(生成)标定图在世界坐标中的坐标 #
    world_point = np.zeros((x_nums * y_nums, 3), np.float32)  # 生成x_nums*y_nums个坐标，每个坐标包含x,y,z三个元素
    world_point[:, :2] = np.mgrid[:x_nums, :y_nums].T.reshape(-1, 2)  # mgrid[]生成包含两个二维矩阵的矩阵，每个矩阵都有x_nums列,y_nums行.T矩阵的转置
    world_point = world_point * size

    axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)  # 设置世界坐标的坐标
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # 设置角点查找限制

    image = cv2.imread(photo_path)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # 查找角点 #
    ok, corners = cv2.findChessboardCorners(
        gray,
        (x_nums, y_nums),
    )
    if ok:
        exact_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)  # 获取更精确的角点位置

        # 获取外参 #
        _, rvec, tvec, inliers = cv2.solvePnPRansac(world_point, exact_corners, mtx, dist)
        R = np.zeros((3, 3), dtype=np.float64)
        cv2.Rodrigues((rvec[0][0], rvec[1][0], rvec[2][0]), R)

        # 可视化角点 #
        # imgpts, jac = cv2.projectPoints(axis, rvec, tvec, mtx, dist)
        # img = draw(image, corners, imgpts)
        # cv2.imshow(windows_name, img)

        return R, tvec


X = []
Y = []
Z = []
R11, R12, R13 = [], [], []
R21, R22, R23 = [], [], []
R31, R32, R33 = [], [], []
R_result = []
T_result = []

# 标定图像保存路径 #
RGB_photo_path = "../../camera_calibration/save_checkboard_img/RGB/*"  # RGB图像保存路径
IR_photo_path = "../../camera_calibration/save_checkboard_img/IR/*"  # IR图像保存路径
Depth_photo_path = "../..camera_calibration/save_checkboard_img/Depth/*"  # Depth图像保存路径


if __name__ == "__main__":
    RGB_images = glob.glob(RGB_photo_path)
    IR_images = glob.glob(IR_photo_path)
    Depth_images = glob.glob(Depth_photo_path)

    for i, RGB_image_path in enumerate(RGB_images):
        R_rgb, T_rgb = calibration_photo(RGB_image_path, RGB_mtx, RGB_dist, "RGB")
        R_ir, T_ir = calibration_photo(IR_images[i], IR_mtx, IR_dist, "IR")

        R_result = R_rgb.dot(np.linalg.inv(R_ir))
        T_result = T_rgb - R_result.dot(T_ir)

        # 为的是多组平均 #
        R11.append(R_result[0][0])
        R12.append(R_result[0][1])
        R13.append(R_result[0][2])
        R21.append(R_result[1][0])
        R22.append(R_result[1][1])
        R23.append(R_result[1][2])
        R31.append(R_result[2][0])
        R32.append(R_result[2][1])
        R33.append(R_result[2][2])

        X.append(T_result[0])
        Y.append(T_result[1])
        Z.append(T_result[2])

    R_result = np.array([[np.mean(R11), np.mean(R12), np.mean(R13), 0.0], [np.mean(R21), np.mean(R22), np.mean(R23), 0.0], [np.mean(R31), np.mean(R32), np.mean(R33), 0.0], [0.0, 0.0, 0.0, 1.0]])

    # 剔除多少个离群点 #
    # del_num = 50
    # test = np.argsort(abs(R12 - np.mean(R12)))[-del_num:]
    # test = np.argsort(abs(R23 - np.mean(R23)))[-del_num:]
    # for i in range(del_num):
    #     os.remove(RGB_images[test[-(i+1)]])
    #     os.remove(IR_images[test[-(i+1)]])
    #     os.remove(Depth_images[test[-(i+1)]])
    #     print(RGB_images[test[-(i+1)]])

    T_result = np.array([[1, 0, 0, np.mean(X) / 100.0], [0, 1, 0, np.mean(Y) / 100.0], [0, 0, 1, np.mean(Z) / 100.0], [0, 0, 0, 1]])

    M = R_result.dot(T_result)

    # 将内参转成齐次的4*4矩阵 #
    RGB_mtx = np.insert(RGB_mtx, 3, values=np.array([[0, 0, 0]]), axis=0)
    RGB_mtx = np.insert(RGB_mtx, 3, values=np.array([[0, 0, 0, 1]]), axis=1)
    IR_mtx = np.insert(IR_mtx, 3, values=np.array([[0, 0, 0]]), axis=0)
    IR_mtx = np.insert(IR_mtx, 3, values=np.array([[0, 0, 0, 1]]), axis=1)

    # 真实值 #
    # RGB_mtx = np.array([[1178.7329083650782, 0.0, 640, 0.0], [0.0, 1178.7329083650782, 360, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    # IR_mtx = np.array([[1178.7329083650782, 0.0, 640, 0.0], [0.0, 1178.7329083650782, 360, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

    W = RGB_mtx.dot(M).dot(np.linalg.inv(IR_mtx))

    # print(W)
    print("%f, %f, %f, %f, %f, %f, %f, %f" % (W[0][0], W[0][1], W[0][2], W[0][3], W[1][0], W[1][1], W[1][2], W[1][3]))  # ir2rgb

    file = r"./Registration_matrix.txt"
    with open(file, "w+") as f:
        str = ("%f, %f, %f, %f, %f, %f, %f, %f") % (W[0][0], W[0][1], W[0][2], W[0][3], W[1][0], W[1][1], W[1][2], W[1][3])
        f.write(str)

    # W_inv = np.linalg.inv(W)
    # print("%f, %f, %f, %f, %f, %f, %f, %f" % (W_inv[0][0], W_inv[0][1], W_inv[0][2], W_inv[0][3], W_inv[1][0], W_inv[1][1], W_inv[1][2], W_inv[1][3]))
