#from gen_utils import *
import squeezedet as nn

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image
import numpy as np

import pickle
import random


IOU_THRESH = 0.5
NN_PROB_THRESH = 0.5

def get_area_cap(b1, b2):
    x1_c, y1_c, l1, w1 = b1
    x2_c, y2_c, l2, w2 = b2

    left_1 = x1_c - (l1/2)
    right_1 = x1_c + (l1/2)
    top_1 = y1_c - (w1/2)
    bot_1 = y1_c + (w1/2)
    left_2 = x2_c - (l2/2)
    right_2 = x2_c + (l2/2)
    top_2 = y2_c - (w2/2)
    bot_2 = y2_c + (w2/2)

    left_cap = max(left_1, left_2)
    right_cap = min(right_1, right_2)
    top_cap = max(top_1, top_2)
    bot_cap = min(bot_1, bot_2)

    area_1 = l1*w1
    area_2 = l2*w2
    area_cap = (right_cap-left_cap)*(bot_cap-top_cap)

    return max(area_cap,0)


def iou(b1, b2):

    x1_c, y1_c, l1, w1 = b1
    x2_c, y2_c, l2, w2 = b2

    area_1 = l1*w1
    area_2 = l2*w2

    area_cap = get_area_cap((x1_c, y1_c, l1, w1), (x2_c, y2_c, l2, w2))

    return area_cap/float(area_1+area_2-area_cap)


def box_2_kitti_format(box):
    '''Transform box for KITTI label format'''
    x = box[0]
    y = box[1]
    w = box[2]
    h = box[3]
    left = int(x - w/2)
    right = int(x + w/2)
    top = int(y - h/2)
    bot = int(y + h/2)
    return [left,top,right,bot]

def kitti_2_box_format(label):

    '''Transform KITTI label format to box'''
    xl = label[0]
    yt = label[1]
    xr = label[2]
    yb = label[3]
    w = xr - xl
    h = yb - yt
    xc = int(xl + w/2)
    yc = int(yt + h/2)
    return [xc, yc, w, h]

def read_label(file_name):
    '''Read label from file (KITTI format)'''
    with open(file_name, "r") as f:
        content = f.readlines()
    content = [c.strip().split(" ") for c in content]
    labels = []
    for c in content:
        if c[0] == 'Car':
            labels += [[float(c[4]), float(c[5]), float(c[6]), float(c[7])]]

    return labels

def read_image_set(image_set):
    '''Read image names from image set'''
    with open(image_set, "r") as f:
        content = f.readlines()
    images = [c.strip() for c in content]

    gt_labels = dict()
    for i in images:
        labels = read_label(PREFIX_LABELS + i + '.txt')

        if labels != []:
            label_boxes = [ kitti_2_box_format(l) for l in labels ]
            gt_labels[i] = label_boxes

    return gt_labels


def predict(net, image_set):
    '''Run neural net on images from image_set'''

    with open(image_set, "r") as f:
        content = f.readlines()
    images = [c.strip() for c in content]

    predictions = dict()
    for i in images:
        file_name = PREFIX_IMAGES + i + '.png'
        pred = nn.classify(file_name, net, NN_PROB_THRESH)

        boxes = []
        for p in pred:
            if p[0] == 0:
                boxes += [ list(p[2]) ]

        for box in boxes:
            # Matrix
            #scale_x = 1.5336
            #scale_y = 2.7396
            # Scenic
            scale_x = 1.5385
            scale_y = 3.125
            box[0] = int(box[0]*scale_x)
            box[1] = int(box[1]*scale_y)
            box[2] = int(box[2]*scale_x)
            box[3] = int(box[3]*scale_y)

        predictions[i] = boxes

    return predictions


def average_precision(gt, prediction, iou_thresh):
    '''Average precision of prediction'''
    alread_detected = [False]*len(gt)
    tp = 0
    fp = 0
    fn = 0

    if not(prediction):
        return 0, 0

    for pred in prediction:
        detect = False
        for i in range(len(gt)):
            #print(iou(pred, gt[i]))
            if iou(pred, gt[i]) > iou_thresh:
                detect = True
                if not alread_detected[i]:
                    tp += 1
                    alread_detected[i] = True
                else:
                    fp += 1
        if not detect:
            fp += 1
    fn = alread_detected.count(False)
    ap = tp/float(tp+fp)
    recall = tp/float(tp+fn)
    return ap, recall


def plot_boxes(image, gt, pred):
    '''Plot gt and prediction boxes'''
    im = np.array(Image.open(PREFIX_IMAGES + image + '.png'), dtype=np.uint8)
    fig,ax = plt.subplots(1)
    ax.imshow(im)
    for box in gt:
        rect = patches.Rectangle((box[0]-box[2]/2, box[1]-box[3]/2),box[2],box[3],linewidth=1,edgecolor='b',facecolor='none')
        ax.add_patch(rect)
    for box in pred:
        rect = patches.Rectangle((box[0]-box[2]/2, box[1]-box[3]/2),box[2],box[3],linewidth=1,edgecolor='r',facecolor='none')
        ax.add_patch(rect)
    plt.show()

def plot_results(x,ys):
    plt.plot(x, ys)
    plt.show()


def get_ap_rec(res):
    '''Get check points, avg precision, and recall from results'''
    cps = []
    aps = []
    recs = []
    for cp in res:
        (ap,rec) = res[cp]
        cps += [cp]
        aps += [ap]
        recs += [rec]
    order = np.argsort(cps)
    x = np.array(cps)[order]
    y1 = np.array(aps)[order]
    y2 = np.array(recs)[order]
    return x, y1, y2



def eval_set(net, image_set, verbose=False):
    '''Evaluate net on image_set'''

    gt_labels = read_image_set(image_set)
    predictions = predict(net, image_set)

    tot_ap = 0
    tot_rec = 0

    for image in gt_labels:

        try:
            gt = gt_labels[image]
            pred = predictions[image]

            ap, rec = average_precision(gt, pred, IOU_THRESH)
            tot_ap += ap
            tot_rec += rec

            if verbose:
                print(image + ': ' + str(ap) + ', ' + str(rec) )
                print(gt)
                print(pred)
                plot_boxes(image, gt,pred)
        except:
            print("ISSUE WITH IOU")

    tot_ap = tot_ap/float(len(gt_labels))
    tot_rec = tot_rec/float(len(gt_labels))

    return tot_ap, tot_rec


def eval_train(checkpoint_dir, checkpoint_list, image_set):
    '''Evaluate training on list of stored checkpoints'''
    results = dict()
    for cp in checkpoint_list:
        cp_path = checkpoint_dir + 'model.ckpt-' + str(cp)
        print('Evaluating: ' + cp_path)
        net = nn.init(cp_path)
        ap, recall = eval_set(net, image_set)
        results[cp] = (ap,recall)
    return results


def gen_augmented_set(image_set, i_start, i_end, n):
    '''Augment images_set with n images with indices are between i_start and i_end'''

    for i in range(1,4):

        PREFIX = 'm_2_' + str(i) + '_'

        arr = range(i_start,i_end)
        random.shuffle(arr)

        arr = arr[0:n]

        with open(image_set, 'a') as f:
            for a in arr:
                f.write(PREFIX + str(a).zfill(6) + '\n')


def multiple_checkpoint_eval():

    checkpoint_list = range(0,5250,250)

    #for i in range(1,4):
    for j in range(1,4):
        #checkpoint_dir = './data/train_' + str(i) + '/' + 'checkpoint_' + str(i) + '_' + str(j) + '/train/'
        checkpoint_dir = './data/train_0/' + 'checkpoint' + '/train/'
        res = eval_train(checkpoint_dir, checkpoint_list, image_set)
        pickle.dump( res, open( "save_mis_05_" + str(j) +".p", "wb" ) )
        #pickle.dump( res, open( "save_mis_"+ str(i) + "_" + str(j) +".p", "wb" ) )


def avg_eval(result_list):
    '''Load results and average them'''
    all_ap = []
    all_rec = []
    for res_l in result_list:
        res = pickle.load( open( res_l, "rb" ) )
        cp, ap, rec = get_ap_rec(res)
        all_ap += [ap]
        all_rec += [rec]
    return cp, np.mean(all_ap,axis=0), np.mean(all_rec,axis=0)



#CHECKPOINT = '/home/tommaso/scenicEx/data/scenic/checkpoints/train/model.ckpt-5500'
#CHECKPOINT = '/home/tommaso/scenicEx/data/matrix/checkpoints_overlap_250/train/model.ckpt-32670'
#CHECKPOINT = '/home/tommaso/scenicEx/data/matrix/checkpoints/train/model.ckpt-5250'
CHECKPOINT = '/home/tommaso/squeezeDet/data/webots/train/checkpoints/train/model.ckpt-1150'
FILE = '/home/tommaso/squeezeDet/data/webots/test/images/1003.png'


PREFIX = '/home/tommaso/scenicEx/data/matrix/'
PREFIX_LABELS = PREFIX + 'labels/'
PREFIX_IMAGES = PREFIX + 'images/'

image_set = '/home/tommaso/scenicEx/data/matrix/ImageSets/test_overlap.txt'

net = nn.init(CHECKPOINT)
pred = nn.classify(FILE, net, NN_PROB_THRESH)
print(pred)
#ap, recall = eval_set(net, image_set)


# avg_precs = []
# avg_recs = []
# max_precs = []
# max_recs = []
#
# for i in range(1,9,1):
#         precs = []
#         recs = []
#         for j in range(4950,5020,10):
#             checkpoint_path = '/home/tommaso/scenicEx/data/matrix/checkpoints_overlap_250/checkpoints_overlap_250_' + str(i) + '/train/model.ckpt-' + str(j)
#             print(checkpoint_path)
#             net = nn.init(checkpoint_path)
#             prec, rec = eval_set(net, image_set, verbose=False)
#             precs += [prec]
#             recs += [rec]
#         avg_precs += [np.mean(precs)]
#         avg_recs += [np.mean(recs)]
#         max_precs += [np.max(precs)]
#         max_recs += [np.max(recs)]
#
#
# print('Avg prec: ' + str(np.mean(avg_precs)))
# print('Avg rec: ' + str(np.mean(avg_recs)))
# print('Max prec: ' + str(np.max(max_precs)))
# print('Max rec: ' + str(np.max(max_recs)))
