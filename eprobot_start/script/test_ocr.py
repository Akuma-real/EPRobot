# -*- coding: utf-8 -*-
import cv2
from PIL import Image
import pytesseract

# 设置 Tesseract 路径
pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

# 图像文件列表
image_files = [
    '/home/EPRobot/robot_ws/src/eprobot_start/script/aera1.jpg',
    '/home/EPRobot/robot_ws/src/eprobot_start/script/aera2.jpg',
    '/home/EPRobot/robot_ws/src/eprobot_start/script/aera3.jpg',
    '/home/EPRobot/robot_ws/src/eprobot_start/script/aera4.jpg'
]

for image_file in image_files:
    # 打开图像
    img = Image.open(image_file)

    # 转换为灰度图像
    img_gray = img.convert('L')

    # 进行图像预处理（例如二值化、去噪、增强对比度等）

    # 使用 pytesseract 进行 OCR，并设置语言选项为英语
    text = pytesseract.image_to_string(img_gray, lang='eng')

    # 打印结果
    print('Image file:', image_file)
    print('Recognized text:', text)
    print('-------------------------')
