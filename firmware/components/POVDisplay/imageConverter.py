import sys
from PIL import Image
from numpy import asarray

image = Image.open(sys.argv[1])
data = asarray(image)
output = "const unsigned int sprite [] = {\n"
for row in data:
    for pixel in row:
        output += "0x{:02x}{:02x}{:02x}{:02x}, ".format(pixel[3], pixel[0], pixel[1], pixel[2])
output = output[:-2]
output += "\n};"
print(output)