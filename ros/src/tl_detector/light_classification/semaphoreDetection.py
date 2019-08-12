import numpy as np
import cv2

# ------------------------------------------------------------------------------------------------------------
def analizeImage(binImg):
    connectivity = 8  # or whatever you prefer
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binImg, connectivity, cv2.CV_32S)

    imageSegment = cv2.cvtColor(binImg, cv2.COLOR_GRAY2RGB)

    cnt = 0
    for bb in stats:
        x, y, w, h, area = bb
        if imageSegment.shape[1] != w and imageSegment.shape[0] != h:
            cv2.rectangle(imageSegment, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cnt = cnt + 1

    #cv2.imshow('image', imageSegment)
    #cv2.waitKey(0)

    return cnt

# ------------------------------------------------------------------------------------------------------------
def threshold(image, thrs):
    image[image <= thrs] = 0
    image[image >= thrs] = 255
    image = np.uint8(image)
    return image

# ------------------------------------------------------------------------------------------------------------
def findSemaphore(img):

    dim = (int(img.shape[1] / 2), int(img.shape[0] / 2))
    imResized = cv2.resize(img, dim)

    # Split the color
    b, g, r = cv2.split(imResized)

    # Convert to integer
    green = np.int32(g)
    red = np.int32(r)
    blue = np.int32(b)

    # Apply the color enhance
    redLight = (blue - green) + 2 * (red - green)
    greenLight = (green - red)
    ambarLight = (green - blue) - greenLight - 0.5*redLight

    # Apply threshold
    greenBin = threshold(greenLight, 125)
    redBin = threshold(redLight, 125)
    ambarBin = threshold(ambarLight, 125)

    # Detect the number of Semaphores (Red, Green, Ambar)
    numGreen = analizeImage(greenBin)
    numRed = analizeImage(redBin)
    numAmbar = analizeImage(ambarBin)

    return numRed, numGreen, numAmbar
# ------------------------------------------------------------------------------------------------------------
