import cv2
import numpy as np

###### CONSTANT VALUE CONFIGURATION ######
#COLOR MASK
lower = np.array([150, 128, 30])
upper = np.array([180, 255, 255])

#AREA and DISTANCE
AREA_REFERENCE = 8500  # [px]
DIST_REFERENCE = 1000   # [mm]
##########################################

def main():
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        # get a frame
        _, frame = cap.read()
        height, width, channels = frame.shape

        # BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        # calc mask
        target = analysis_blob(mask)

        # get center
        x = int(target["center"][0])
        y = int(target["center"][1])

        cv2.circle(frame, (x,y), 4, 255, 2, 4)

        cv2.imshow('frame' , frame)
        cv2.imshow('mask', mask)

        # get areas and calc dist
        area = target["area"]
        if area != 0:
            rate = AREA_REFERENCE/area
        else:
            rate = np.Infinity

        dist = DIST_REFERENCE * np.sqrt(rate)
        print(dist)

        # Esc
        if cv2.waitKey(5) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


def analysis_blob(binary_img):
    if cv2.countNonZero(binary_img) <= 0:
        maxblob = {}

        # 面積最大ブロブの各種情報を取得
        maxblob["upper_left"] = {0, 0}  # 左上座標
        maxblob["width"] = 0  # 幅
        maxblob["height"] = 0  # 高さ
        maxblob["area"] = 0  # 面積
        maxblob["center"] = {0, 0}  # 中心座標

        return maxblob

    # 2値画像のラベリング処理
    label = cv2.connectedComponentsWithStats(binary_img)

    # ブロブ情報を項目別に抽出
    n = label[0] - 1
    data = np.delete(label[2], 0, 0)
    center = np.delete(label[3], 0, 0)

    # ブロブ面積最大のインデックス
    max_index = np.argmax(data[:, 4])

    # 面積最大ブロブの情報格納用
    maxblob = {}

    # 面積最大ブロブの各種情報を取得
    maxblob["upper_left"] = (data[:, 0][max_index], data[:, 1][max_index])  # 左上座標
    maxblob["width"] = data[:, 2][max_index]  # 幅
    maxblob["height"] = data[:, 3][max_index]  # 高さ
    maxblob["area"] = data[:, 4][max_index]  # 面積
    maxblob["center"] = center[max_index]  # 中心座標

    return maxblob


if __name__ == '__main__':
    main()
