from __future__ import division
import numpy as np
from six.moves import range
import PIL
from PIL import ImageEnhance


def augment_img(img, crop_size=(64, 48)):
    width, height = img.size
    max_crop_left = img.size[0] - crop_size[0]
    max_crop_upper = img.size[1] - crop_size[1]
    left = np.random.choice(max_crop_left)
    upper = np.random.choice(max_crop_upper)
    img = img.crop([left, upper, left + crop_size[0], upper + crop_size[1]])
    brightness = ImageEnhance.Brightness(img)

    img = brightness.enhance(np.random.uniform(0.75, 2.5))
    contrast = ImageEnhance.Contrast(img)
    img = contrast.enhance(np.random.uniform(0.75, 2.5))
    return img


def augment_batch(batch):
    """Applies random augmentation to the batch."""
    images = [PIL.Image.fromarray(x) for x in batch["image"]]
    images = [augment_img(img) for img in images]
    batch_aug = {'image': np.stack([np.array(img) for img in images])}
    for k, v in batch.items():
        if k != 'image':
            batch_aug[k] = v
    return batch_aug


def crop_img(img, crop_size=(64, 48), resize=(80, 60)):
    img = img.convert('L')
    if img.size != resize:
        img = img.resize(resize, resample=PIL.Image.BILINEAR)
    img = img.crop(
        map(
            int,
            [0.5 * (resize[0] - crop_size[0]),
             0.5 * (resize[1] - crop_size[1]),
             0.5 * (resize[0] + crop_size[0]),
             0.5 * (resize[1] + crop_size[1])])
    )
    return np.asarray(img)


def crop_batch(batch):
    """Crops the batch at the center and does not use augmentations."""
    images = [PIL.Image.fromarray(x) for x in batch["image"]]
    images = [crop_img(img) for img in images]
    batch_aug = {'image': np.stack([np.array(img) for img in images])}
    for k, v in batch.items():
        if k != 'image':
            batch_aug[k] = v
    return batch_aug
