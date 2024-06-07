import cv2
from src.image import ImageProcessor
import shutil
import os

path = "imgs\\Imagenes\\"

for folder in ["C", "F", "H", "O", "P", "S", "U", "None"]:
    if not os.path.exists("imgs\\" + folder):
        os.makedirs("imgs\\" + folder)
    else:
        for f in os.listdir("imgs\\" + folder):
            os.remove("imgs\\" + folder + "\\" + f)

imageProcessor = ImageProcessor()
for f in os.listdir(path):
    img = cv2.imread(path + f)
    result = imageProcessor.procesar(img)
    
    if result == "C":
        shutil.copy(path + f, "imgs\\C\\" + f)
    elif result == "F":
        shutil.copy(path + f, "imgs\\F\\" + f)
    elif result == "H":
        shutil.copy(path + f, "imgs\\H\\" + f)
    elif result == "O":
        shutil.copy(path + f, "imgs\\O\\" + f)
    elif result == "P":
        shutil.copy(path + f, "imgs\\P\\" + f)
    elif result == "S":
        shutil.copy(path + f, "imgs\\S\\" + f)
    elif result == "U":
        shutil.copy(path + f, "imgs\\U\\" + f)
    else:
        shutil.copy(path + f, "imgs\\None\\" + f)