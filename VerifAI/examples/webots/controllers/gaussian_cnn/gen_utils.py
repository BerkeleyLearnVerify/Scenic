from image_generation_utils import *
from image_mod_gen_utils import *
from analysis_nn import *
from update_library import *

lib = update_library()

def gen_image(sample):
    fg = []
    for s in range(len(sample[1])):
        fg += [fg_obj(fg_id=sample[1][s], x=sample[2][s], y=sample[3][s])]
    return gen_comp_img(lib, fg, bg_id=sample[0][0], brightness=sample[4][0], sharpness=sample[5][0], contrast=sample[5][0], color=sample[5][0])

def scale_sample(sample, sampling_domain):
    '''Scale a [0,1] sample in a given domain'''
    for i in range(len(sample)):
        sample[i] = sample[i]*(sampling_domain[i][1] - sampling_domain[i][0]) + sampling_domain[i][0]
    return sample



def random_sample(typ, domain, n_samples):

    sample = []
    for _ in range(n_samples):
        if typ == 'float':
            r = random.random()*(domain[1]-domain[0])+domain[0]
        elif typ == 'int':
            r = random.choice(range(domain[0],domain[1]))
        else:
            print('Error')
        sample.append(r)
    return sample

def random_config(domains, n_cars):
#     print("domains is : ", domains)# [[52, 72], [1, 19], [0, 1], [0.35, 1], [0.8, 1.2], [0.7, 1]]
#     print("n_cars is : ", n_cars)
    # Scene and cars
    sample = [random_sample('int',domains[0],1)]
    sample += [random_sample('int',domains[1],n_cars)]
    # Modifications
    x_sample = []
    y_sample = []
    if n_cars > 0:
        step_x = float(domains[2][1])/n_cars
        step_y = float(domains[3][1])/n_cars

        base_x = 0
        base_y = 0
        for _ in range(n_cars):
            x_sample.append(random.uniform(base_x, base_x+step_x))
            y_sample.append(random.uniform(base_y, base_y+step_y))
            base_x += step_x
            base_y += step_y

    shuffle(x_sample)
    y_sample.sort(reverse=True)
    sample += [x_sample]
    sample += [y_sample]

    sample += [random_sample('float',domains[4],1)]
    sample += [random_sample('float',domains[5],1)]

    return sample


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


# def get_area_cap((x1_c, y1_c, l1, w1), (x2_c, y2_c, l2, w2)):
#     left_1 = x1_c - (l1/2)
#     right_1 = x1_c + (l1/2)
#     top_1 = y1_c - (w1/2)
#     bot_1 = y1_c + (w1/2)
#     left_2 = x2_c - (l2/2)
#     right_2 = x2_c + (l2/2)
#     top_2 = y2_c - (w2/2)
#     bot_2 = y2_c + (w2/2)
#
#     left_cap = max(left_1, left_2)
#     right_cap = min(right_1, right_2)
#     top_cap = max(top_1, top_2)
#     bot_cap = min(bot_1, bot_2)
#
#     area_1 = l1*w1
#     area_2 = l2*w2
#     area_cap = (right_cap-left_cap)*(bot_cap-top_cap)
#
#     return max(area_cap,0)


# def iou((x1_c, y1_c, l1, w1), (x2_c, y2_c, l2, w2)):
#
#     area_1 = l1*w1
#     area_2 = l2*w2
#
#     area_cap = get_area_cap((x1_c, y1_c, l1, w1), (x2_c, y2_c, l2, w2))
#
#     return area_cap/float(area_1+area_2-area_cap)
#
#
# def iomin((x1_c, y1_c, l1, w1), (x2_c, y2_c, l2, w2)):
#     area_1 = l1*w1
#     area_2 = l2*w2
#     area_cap = get_area_cap((x1_c, y1_c, l1, w1), (x2_c, y2_c, l2, w2))
#
#     min_area = min(area_1, area_2)
#
#     return area_cap / float(min_area)

def save_image(img, file_name, path_data_set):
    '''Save image and label'''

    img_file_name = path_data_set + 'images/' + file_name + '.png'
    img.save(img_file_name)


def save_label(ground_boxes, file_name, path_data_set):
    '''Save label'''

    f = open(path_data_set + 'labels/' + file_name + '.txt', 'w')

    if len(ground_boxes) > 0:
        for box in ground_boxes[:-1]:
            label = [0,0,0] + box_2_kitti_format(box) + [0,0,0,0,0,0,0]
            label = list(map(str, label))
            label = " ".join(label)
            label  = "Car " + label + "\n"
            f.write(label)  # python will convert \n to os.linesep
        label = [0,0,0] + box_2_kitti_format(ground_boxes[-1]) + [0,0,0,0,0,0,0]
        label = list(map(str, label))
        label = " ".join(label)
        label  = "Car " + label
        f.write(label)  # python will convert \n to os.linesep
    f.close()


def pad_sample(conf):
    '''Covert config to list'''
    MAX_NUM_CARS = 3
    pad = MAX_NUM_CARS - len(conf[1])

    pt = conf[0]                # background
    pt += (conf[1] + [-1]*pad)  # car models
    pt += (conf[2] + [-1]*pad)  # x
    pt += (conf[3] + [-1]*pad)  # y
    pt += conf[4]               # image params
    pt += conf[5]
    pt += conf[6]
    pt += conf[7]

    return pt

def check_perspective(xs, ys, x_eps, y_eps):
    '''Check distance between sampled points'''
    for yi in range(len(ys)):
        for yj in range(yi+1,len(ys)):
            if abs(ys[yi] - ys[yj]) < y_eps:
                if abs(xs[yi] - xs[yj]) < x_eps:
                    return False
    return True


def pred_2_detect(preds, gt, iou_tresh):
    '''Assign predictions to detections'''
    detetctions = []
    for p in preds:
        detects = []
        for g in gt:
            detects += [iou(p, g) > iou_tresh]
        detetctions += [detects]
    assigns = []
    for d in detetctions:
        a = -1
        for i in range(len(d)):
            if d[i]:
                a = i
        assigns += [a]

    assigns = duplicate_false_positive(assigns, len(gt))
    return assigns

def duplicate_false_positive(detects, n_gt):
    '''Correct double false positives'''
    for i in range(n_gt):
        first_occ = True
        for j in range(len(detects)):
            if i == detects[j]:
                if first_occ:
                    first_occ = False
                else:
                    detects[j] = -1
    return detects


def prec_rec(detects, n_gt):
    '''Compute precision and recall'''

    tp = sum(d != -1 for d in detects)
    fp = sum(d == -1 for d in detects)
    fn = sum(g not in detects for g in range(n_gt))

    prec = tp/float(tp+fp)
    rec = tp/float(tp+fn)

    return prec, rec


def precision_recall(preds, gt_boxes, iou_tresh):
    detects = pred_2_detect(preds, gt_boxes, iou_tresh)
    return prec_rec(detects, len(gt_boxes))
