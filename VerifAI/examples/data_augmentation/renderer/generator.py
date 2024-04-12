"""Image composition and modification primitives"""

from PIL import Image, ImageEnhance
from renderer.utils import *

def genImage(lib, sample):
    """Generate image, labels, and meta data from a sample"""

    # Sort samples by depth
    cars = sorted(sample.cars, key=lambda car:
            -car.yPos[0])
    fg = []
    for car in cars:
        fg += [fgObj(fgId=car.carID,
                     x=car.xPos[0],
                     y=car.yPos[0])]

    return genCompImg(
        lib,
        fg,
        bgId=sample.backgroundID,
        brightness=sample.brightness[0],
        sharpness=sample.sharpness[0],
        contrast=sample.contrast[0],
        color=sample.color[0])


def scaleImg(img, scale):
    return img.resize((np.array(img.size) * scale).astype(int))


def scaleGetLoc(img, scale, centroid):
    scaledImg = scaleImg(img, scale)
    topRightLoc = (
        np.array(centroid) - np.array(scaledImg.size) * 0.5).astype(int)
    return scaledImg, topRightLoc


def modifyImageBscc(imageData, brightness, sharpness, contrast, color):
    """Update with brightness, sharpness, contrast and color."""

    brightnessMod = ImageEnhance.Brightness(imageData)
    imageData = brightnessMod.enhance(brightness)

    sharpnessMod = ImageEnhance.Sharpness(imageData)
    imageData = sharpnessMod.enhance(sharpness)

    contrastMod = ImageEnhance.Contrast(imageData)
    imageData = contrastMod.enhance(contrast)

    colorMod = ImageEnhance.Color(imageData)
    imageData = colorMod.enhance(color)

    return imageData


def genCompImg(library,
               fgObjects,
               bgId=0,
               brightness=1.,
               sharpness=1.,
               contrast=1.,
               color=1.):
    """Compose an image from a sample."""

    background = library.backgroundObjects[bgId]
    scalingFactor = background.scaling
    backgroundCopy = background.image.copy()

    # remove alpha channel from background (if present)
    if backgroundCopy.mode in (
            'RGBA', 'LA') or (backgroundCopy.mode == 'P'
                              and 'transparency' in backgroundCopy.info):
        backgroundNoAlpha = Image.new("RGB", backgroundCopy.size,
                                      (255, 255, 255))
        backgroundNoAlpha.paste(
            backgroundCopy,
            mask=backgroundCopy.split()[3])  # 3 is the alpha channel
    else:
        backgroundNoAlpha = backgroundCopy

    # Add foreground images
    boxes = []
    for i, fgi in zip(range(len(fgObjects)), fgObjects):
        x, y, fg = fgi.x, fgi.y, fgi.fgId
        scaleFg = y * (
            scalingFactor.back - scalingFactor.front) + scalingFactor.front
        sampleConvSpace = ld2bbSample(sample=[x, y], h=background.homographyH)
        foreground = library.foregroundObjects[fg]
        scaledImg, topRightLoc = scaleGetLoc(foreground.image, scaleFg,
                                             sampleConvSpace)

        # paste car
        backgroundNoAlpha.paste(scaledImg, tuple(topRightLoc), scaledImg)

        # store labels
        intCentroid = list(sampleConvSpace.astype(int))
        listSize = list(scaledImg.size)

        boxes.append(intCentroid + listSize)

    modifImg = modifyImageBscc(
        imageData=backgroundNoAlpha,
        brightness=brightness,
        sharpness=sharpness,
        contrast=contrast,
        color=color)

    return modifImg, boxes