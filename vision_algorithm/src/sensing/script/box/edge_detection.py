import cv2
import numpy as np
from numba import jit
import time
from numba import cuda
import math
from matplotlib import pyplot as plt
from config import parser
# from skimage.draw import line

params = parser.parse_args()
PREPROCESS_THRESHOLD_DEFAULT = params.PREPROCESS_THRESHOLD_DEFAULT
TOPK = 100
RATIO2MAX = 0.05
BIN_SIZE = 3

@cuda.jit
def hough_transformation_gpu_impl(edges_pts, accumulator_array, pmax, thetas, n_pts):
    theta_idx = cuda.threadIdx.y
    if theta_idx >= 180:
        return
    step = cuda.blockDim.x * cuda.gridDim.x
    for i in range(math.ceil(n_pts[0] / step)):
        pts_idx = cuda.threadIdx.x + cuda.blockIdx.x * cuda.blockDim.x + step * i
        if pts_idx >= n_pts[0]:
            return
        pts = edges_pts[pts_idx]
        theta = thetas[theta_idx]
        p = pts[0] * math.cos(theta * np.pi / 180.0) + pts[1] * math.sin(theta * np.pi / 180.0)
        cuda.atomic.add(accumulator_array[theta_idx], int(p) + int(pmax[0] / 2.), 1)
    return

@cuda.jit()
def find_local_max_gpu(accumulator_array, accumulator_array_out, n):
    theta_idx = cuda.threadIdx.y
    if theta_idx >= 179 or theta_idx == 0:
        return
    step = cuda.blockDim.x * cuda.gridDim.x
    for i in range(math.ceil(n[0] / step)):
        p_idx = cuda.threadIdx.x + cuda.blockIdx.x * cuda.blockDim.x + step * i
        if p_idx >= n[0]:
            return
        if p_idx == 0 or p_idx == n[0] - 1:
            continue
        if accumulator_array[theta_idx][p_idx] >= accumulator_array[theta_idx][p_idx - 1] and \
                accumulator_array[theta_idx][p_idx] >= accumulator_array[theta_idx][p_idx + 1] and \
                accumulator_array[theta_idx][p_idx] >= accumulator_array[theta_idx - 1][p_idx] and \
                accumulator_array[theta_idx][p_idx] >= accumulator_array[theta_idx + 1][p_idx] and \
                accumulator_array[theta_idx][p_idx] >= accumulator_array[theta_idx - 1][p_idx - 1] and \
                accumulator_array[theta_idx][p_idx] >= accumulator_array[theta_idx - 1][p_idx + 1] and \
                accumulator_array[theta_idx][p_idx] >= accumulator_array[theta_idx + 1][p_idx - 1] and \
                accumulator_array[theta_idx][p_idx] >= accumulator_array[theta_idx + 1][p_idx + 1]:
            accumulator_array_out[theta_idx][p_idx] = accumulator_array[theta_idx][p_idx]



@jit(nopython=True)
def hough_transformation_jit(edges, accumulator_array, pmax):
    edge_pts_idx = np.where(edges > 200)
    edge_pts = np.stack((edge_pts_idx[1], edge_pts_idx[0]), axis=-1)
    thetas = np.arange(-89, 91).astype(np.float32)
    p = edge_pts[:, 0].copy().reshape((-1, 1)) * np.cos(thetas.reshape((1, -1)) * np.pi / 180.0) + \
        edge_pts[:, 1].copy().reshape((-1, 1)) * np.sin(thetas.reshape((1, -1)) * np.pi / 180.0)
    p_indices = (np.floor(p).astype(np.int32) + int(round(pmax / 2))).reshape(-1)
    for i, p_index in enumerate(p_indices):
        theta_index = int(thetas[i % thetas.shape[0]]) + 89
        accumulator_array[theta_index][int(p_index)] += 1
    return accumulator_array
### end hough transformation jit ###

def hough_transformation_gpu(accumulator_array, edges, pmax):
    num_max_pts = 200000
    threads_per_block = (2, 256)
    blocks_per_grid = math.ceil(num_max_pts / threads_per_block[0])
    edge_pts_idx = np.where(edges > 200)
    edge_pts = np.stack((edge_pts_idx[1], edge_pts_idx[0]), axis=-1)
    edge_pts_device = cuda.to_device(edge_pts)
    accumulator_array_device = cuda.to_device(accumulator_array)
    pmax_device = cuda.to_device(np.array([pmax]))
    thetas = np.arange(-89, 91).astype(np.float32)
    thetas_device = cuda.to_device(thetas)
    cnt_edge_pts_device = cuda.to_device(np.array([edge_pts.shape[0]]))
    hough_transformation_gpu_impl[blocks_per_grid, threads_per_block](edge_pts_device,
                                        accumulator_array_device, pmax_device, thetas_device, cnt_edge_pts_device)

    cuda.synchronize()
    accumulator_array_out = np.zeros_like(accumulator_array).astype(np.int32)
    accumulator_array_out_device = cuda.to_device(accumulator_array_out)
    n = cuda.to_device(np.array([accumulator_array.shape[1]]))
    find_local_max_gpu[blocks_per_grid, threads_per_block](accumulator_array_device, accumulator_array_out_device, n)
    accumulator_array = accumulator_array_out_device.copy_to_host()
    cuda.synchronize()
    return accumulator_array

def cir_run_avg(data, bin_start, bin_end, bin_size):
    bin_offset_l = bin_size // 2
    bin_offset_r = bin_size - bin_offset_l - 1
    data_cir = np.concatenate(
        [data[data <= 0 + bin_offset_r - 1] + 255, data, data[data >= 255 - bin_offset_l + 1] - 0])
    bins = []
    hist = []
    for i in range(bin_start, bin_end + 1):
        bins.append(i)
        range_l = i - bin_offset_l
        range_r = i + bin_offset_r
        hist.append(np.logical_and(data_cir >= range_l, data_cir < range_r).sum())
    return np.array(bins), np.array(hist)

def approx_contour(edges, peri_ratio=0.02):
    edges_aprrox = np.zeros_like(edges)
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        peri = peri_ratio * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, peri, True)
        cv2.drawContours(edges_aprrox, [approx], -1, (255, 255, 255), 1)
    # cv2.imshow("edges", edges_aprrox)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return edges_aprrox


def egde_detect(img, model_path):
    # img = cv2.imread(img_path, 0)
    edge_detection = cv2.ximgproc.createStructuredEdgeDetection(model_path)
    rgb_im = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    # time1 = time.time()
    rgb_im_binary = np.zeros_like(rgb_im, dtype=np.uint8)
    gray_im = rgb_im[:, :, 0]
    bins, hist = cir_run_avg(gray_im.reshape(-1), 0, 255, 13)
    # plt.plot(bins, hist)
    # plt.show()

    # bins_max_idx = hist.argmax()
    # local_min = -1
    # for i in range(bins_max_idx + 1, bins[-1] + 1):
    #     if hist[i] > hist[i - 1]:
    #         local_min = i
    #         break
    # preprcoess_threshold = PREPROCESS_THRESHOLD_DEFAULT if local_min == -1 else local_min
    preprcoess_threshold = PREPROCESS_THRESHOLD_DEFAULT
    mask = img > preprcoess_threshold
    rgb_im_binary[mask] = 255

    # cv2.imshow('result', rgb_im_binary)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    edges = edge_detection.detectEdges(np.float32(rgb_im_binary) / 255.0)
    orimap = edge_detection.computeOrientation(edges)
    edges = edge_detection.edgesNms(edges, orimap)

    edges *= 255
    edges = edges.astype(np.uint8)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, (5, 5), iterations=6)
    edges = cv2.threshold(edges, 40, 255, cv2.THRESH_BINARY)[1]

    pmax = np.sqrt(np.square(len(edges)) + np.square(len(edges[0])))
    accumulator_array = np.array([[0 for _ in range(2 * int(pmax) + 1)] for _ in range(-89, 91)])
    accumulator_array = hough_transformation_gpu(accumulator_array, edges, pmax)
    if (accumulator_array != 0.).any():
        theta_idx, p_idx = np.where(accumulator_array > 0.)
        accumulator_array = accumulator_array[accumulator_array > 0.]
        sort_idx = accumulator_array.argsort()[::-1]

        accumulator_array = accumulator_array[sort_idx][:TOPK]
        theta_idx = theta_idx[sort_idx][:TOPK]
        p_idx = p_idx[sort_idx][:TOPK]

        mask = accumulator_array > accumulator_array.max() * RATIO2MAX
        theta_idx = theta_idx[mask]
        p_idx = p_idx[mask]

        theta = theta_idx - 89
        p = p_idx - pmax / 2
        bins, hist = cir_run_avg(theta, -89, 90, BIN_SIZE)

        filtered_theta, filtered_p = [], []
        for i in range(len(theta)):
            a_theta = theta[i]
            idx_bins = (a_theta >= bins).sum() - 1
            if a_theta <= 0:
                theta2 = a_theta + 90
            else:
                theta2 = a_theta - 90
            idx_bins2 = (theta2 >= bins).sum() - 1
            if hist[idx_bins] >= 5 and hist[idx_bins2] >= 5:
                filtered_theta.append(a_theta)
                filtered_p.append(p[i])


        result_img = np.zeros_like(img, dtype=np.uint8)
        for a_theta, a_p in zip(filtered_theta, filtered_p):
            if a_theta != 0:
                a_theta_arc = a_theta / 180 * np.pi
                k = - np.cos(a_theta_arc) / np.sin(a_theta_arc)
                b = a_p / np.sin(a_theta_arc)
                y1 = int(b)
                y2 = int(k * edges.shape[1] + b)
                pts1 = (0, y1)
                pts2 = (edges.shape[1], y2)
            else:
                pts1 = (int(a_p), 0)
                pts2 = (int(a_p), edges.shape[0])
            cv2.line(result_img, pts1, pts2, (255, 255, 255), 20)

        result_img_bool = result_img.astype(np.bool)
        edges_bool = edges.astype(np.bool)
        result_img_bool = result_img_bool & edges_bool
        result_img = result_img_bool.astype(np.uint8) * 255
    else:
        result_img = np.zeros_like(img, dtype=np.uint8)

    # print(time.time() - time1)

    # cv2.imshow('result', result_img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    #
    # cv2.imwrite("result3.bmp", result_img)
    return result_img


if __name__ == "__main__":
    egde_detect("../data/leftView5.bmp")
