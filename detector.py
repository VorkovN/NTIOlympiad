# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt
import numpy as np

cargo_amount = {
    'green': 0,
    'red': 0,
    'yellow': 0,
    'blue': 0
}

"""
color_thresholds = {
    'green': np.array([[45, 30, 0], [75, 100, 255]]),
    'red': np.array([[0, 100, 0], [5, 255, 255]]),
    'yellow': np.array([[25, 100, 0], [35, 255, 255]]),
    'blue': np.array([[115, 100, 0], [125, 255, 255]])
}
"""

REFERENCE_COLORS = {
    'green': [87, 128, 77],
    'red': [102, 103, 194],
    'yellow': [91, 214, 216],
    'blue': [130, 117, 73]
}

DEBUG_COLORS = {
    'green': [0, 255, 0],
    'red': [0, 0, 255],
    'yellow': [0, 255, 255],
    'blue': [255, 0, 0]
}

KMEANS_N_COLORS = 5
COLOR_TOLERANCE = 60
DEBUG = True

l = 0

def detect_cargo(img):
    global cargo_amount

    cur_cargo_amount = {
        'green': 0,
        'red': 0,
        'yellow': 0,
        'blue': 0
    }

    # Поиск контуров
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray_img, 100, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    if DEBUG:
        cv2.imshow("Thresh", thresh)

    # Поиск прямоугольников
    rects, shapes = [], []
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.025*cv2.arcLength(cnt, True), True)
        area = cv2.contourArea(approx)
        if len(approx) == 4 and area > 100:
            rects.append(np.reshape(approx, (4, 2)))
            shapes.append(cv2.minAreaRect(cnt))

    cargo_idx = {
        'green': [],
        'red': [],
        'yellow': [],
        'blue': []
    }
    # Выделение доминантного цвета в прямоугольнике и сравнение с референсными цветами посылок
    for i in range(len(shapes)):
        box = cv2.boxPoints(shapes[i]).astype(np.uint8)
        coords_min = np.min(box, axis=0)
        coords_max = np.max(box, axis=0)
        crop_img = img[coords_min[1]:coords_max[1], coords_min[0]:coords_max[0]]
        #cv2.imshow("4", crop_img)

        # Применяем метод k-средних
        pixels = np.float32(crop_img.reshape(-1, 3))
        _, labels, palette = cv2.kmeans(pixels, KMEANS_N_COLORS, None,
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, .1), 10, cv2.KMEANS_RANDOM_CENTERS)
        _, counts = np.unique(labels, return_counts=True)

        dominant = palette[np.argmax(counts)]
        #cv2.imshow("12312"+str(coords_max[0])+"bb", np.full(img.shape, dominant, dtype=np.uint8))

        min_diff = 255
        min_color = ''
        for color in ['green', 'red', 'yellow', 'blue']:
            diff = cv2.norm(dominant.astype(np.uint8) - REFERENCE_COLORS[color])
            if DEBUG:
                print color, diff
            if diff < min_diff:
                min_color = color
                min_diff = diff
        
        if min_diff < COLOR_TOLERANCE:
            cur_cargo_amount[min_color] += 1
            cargo_idx[min_color].append(i)
            if DEBUG:
                print '['+min_color+']'
        if DEBUG:
            print ''

    # Стремимся зафиксировать в одном кадре все грузы
    for k, v in cur_cargo_amount.iteritems():
        if cargo_amount[k] < v:
            cargo_amount[k] = v
    
    if DEBUG:
        print(cur_cargo_amount)
        for color, cargo in cargo_idx.iteritems():
            img = cv2.drawContours(img, [rects[i] for i in range(len(rects)) if i in cargo], -1, DEBUG_COLORS[color], 2)
        cv2.imshow("Img with contours", img)


if __name__ == '__main__':
    detect_cargo(cv2.imread('test.jpg'))

    cv2.waitKey(0)
    cv2.destroyAllWindows()