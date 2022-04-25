import math
from time import time
from typing import List
import random
import cv2 as cv
from cv2 import DESCRIPTOR_MATCHER_BRUTEFORCE
import numpy as np
from scipy import signal
import scipy
import scipy.sparse
from skimage import morphology
import skan
import networkx as nx
from jsonc_parser.parser import JsoncParser
from sklearn import cluster
from urllib3 import Retry

MIN_LINE_LENGTH = 10

def reorderPins(netList):
    resultNetList = []

    for cmp in netList:
        if cmp['component'] in ['OPV', 'S3', 'NPN', 'PNP', 'MFET_N_D', 'MFET_N_E', 'MFET_P_D', 'MFET_P_E', 'JFET_N', 'JFET_P']:
            pins = cmp['pins']
            center = np.asarray([cmp['position']['x'], cmp['position']['y']]) * 0.5 + (np.asarray([pins[0]['x'], pins[0]['y']]) + np.asarray([pins[1]['x'], pins[1]['y']]) + np.asarray([pins[2]['x'], pins[2]['y']])) / 3.0 * 0.5

            minError = float('inf')
            finalPins = pins

            for i in range(3):
                single = np.asarray([pins[i]['x'], pins[i]['y']]) - center
                a = np.asarray([pins[(i + 1) % 3]['x'], pins[(i + 1) % 3]['y']]) - center
                b = np.asarray([pins[(i + 2) % 3]['x'], pins[(i + 2) % 3]['y']]) - center

                singleAngle = math.atan2(single[1], single[0])
                
                angleA = math.atan(math.tan(math.atan2(a[1], a[0]) - singleAngle))
                angleB = math.atan(math.tan(math.atan2(b[1], b[0]) - singleAngle))

                otherAngleError = abs(angleA + angleB)
                error = abs(math.sin(2.0 * singleAngle)) * math.pi / 4.0 + otherAngleError

                if error < minError:
                    minError = error

                    if cmp['component'] == 'OPV':
                        finalPins = [pins[(i + 1) % 3], pins[(i + 2) % 3], pins[i]]
                    else:
                        finalPins = [pins[i], pins[(i + 1) % 3], pins[(i + 2) % 3]]

            cmp['pins'] = finalPins
            resultNetList.append(cmp)
        else:
            resultNetList.append(cmp)
    
    return resultNetList

def scaleNeuralOutValues(neural_out, scale):
    def scale_contained_vector(val):
        if type(val) is dict:
            if 'x' in val:
                val['x'] *= scale
                val['y'] *= scale
                return
            else:
                for v in val.values():
                    scale_contained_vector(v)
        elif type(val) is list:
            for v in val:
                scale_contained_vector(v)
        else:
            return
    for cmp in neural_out:
        scale_contained_vector(cmp)

def isConnectedKnot(position, img):
    """
    Check if junction is connected (with black dot) or is just a crossing of two lines
    position: (x, y) in px
    img: a grayscale opencv image with black background
    """
    PATCH_HALF_SIZE = 15
    patch = img[position[1]-PATCH_HALF_SIZE:position[1]+PATCH_HALF_SIZE, position[0]-PATCH_HALF_SIZE:position[0]+PATCH_HALF_SIZE]
    patch = cv.copyMakeBorder(patch, 3, 3, 3, 3, cv.BORDER_CONSTANT, value=0)
    _, patch = cv.threshold(patch, 127, 255, cv.THRESH_BINARY)

    dist = cv.distanceTransform(patch, cv.DIST_L2, cv.DIST_MASK_3)

    maxima = signal.argrelextrema(dist, np.greater, order=4)
    
    if maxima[0].size == 0:
        return False

    center = PATCH_HALF_SIZE + 3

    dist_to_center = np.square(maxima[0] - center) + np.square(maxima[1] - center)
    x = maxima[1][np.argmin(dist_to_center)]
    y = maxima[0][np.argmin(dist_to_center)]
    # print(dist[y, x])
    # dist_norm = (dist / np.amax(dist) * 255).astype(np.uint8)
    # cv.imshow('d', dist_norm)
    # cv.waitKey(0)

    return dist[y, x] > 2.6

def angleToInterval(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

def pointLinesDistances(point, lines):
    """
    Normal distance between a point and a line if the normal projection lies on the line segment.
    Otherwise distance to nearest endpoint.
    Input:
        point: np.array [x, y]
        lines: np.array [[x1, y1, x2, y2], ...]
    Returns:
        distances, index of closer endpoint
    """
    t = np.sum((point - lines[:, 0:2]) * (lines[:, 2:4] - lines[:, 0:2]), axis=-1) / np.sum((lines[:, 2:4] - lines[:, 0:2])**2, axis=-1)
    t = np.maximum(np.minimum(t, 1), 0)

    projections = lines[:, 0:2] + (lines[:, 2:4] - lines[:, 0:2]) * np.expand_dims(t, -1)
    distances = np.sqrt(np.sum((point - projections)**2, axis=-1))

    return distances, projections, np.where(t < 0.5, 0, 1)

def findPinStart(pinPos, compType, compBox, lines, angles):
    """
    Finds the line best matching the pin prediction of a component.
    Inputs:
        pinPos: np.array [x, y]
        compType: string ID
        compBox: np.array [xmin, ymin, xmax, ymax]
        lines: [[x1, y1, x2, y2], ...]
    Reutrns:
        line index to remove,
        start point,
        end point
    """
    lines = np.asarray(lines)
    horizontal_mask = np.abs(np.abs(np.abs(angles) - np.pi/2) - np.pi/2) < np.pi/8
    vertical_mask = np.abs(np.abs(angles) - np.pi/2) < np.pi/8
    horizontal_lines = lines[horizontal_mask, :]
    vertical_lines = lines[vertical_mask, :]

    # TODO object type specific filtering for more robustness

    if compType == 'PIN':
        if compBox[2] - compBox[0] > compBox[3] - compBox[1]:
            distances, projections, endpoint_indices = pointLinesDistances(pinPos, horizontal_lines)
            i = np.argmin(distances)
            line_idx = np.searchsorted(np.cumsum(horizontal_mask), i + 1)
        else:
            distances, projections, endpoint_indices = pointLinesDistances(pinPos, vertical_lines)
            i = np.argmin(distances)
            line_idx = np.searchsorted(np.cumsum(vertical_mask), i + 1)
    else:
        distances, projections, endpoint_indices = pointLinesDistances(pinPos, lines)
        line_idx = i = np.argmin(distances)
    
    # find end point which is further away from the object center
    center = (compBox[0:2] + compBox[2:4]) / 2.0
    dist_a = np.sqrt(np.sum((lines[line_idx, 0:2] - center)**2))
    dist_b = np.sqrt(np.sum((lines[line_idx, 2:4] - center)**2))
    farther_p_idx = 0 if dist_a > dist_b else 2

    return line_idx, lines[line_idx, (2-farther_p_idx):(2-farther_p_idx)+2], lines[line_idx, farther_p_idx:farther_p_idx+2]

def splitComponents(img, components):
    """
    Split components with white image stripes to ensure a complete seperation of nets.
    """
    SPLIT_PERPENDICULAR = {'I2', 'C', 'I1', 'U1', 'U_AC', 'V', 'S2', 'S1', 'BTN1', 'D_Z', 'U2', 'U3', 'R', 'M', 'D_S', 'L', 'BTN2', 'D', 'C_P', 'BAT', 'LMP', 'A', 'F', 'LED', 'L2'}
    SPLIT_BLOB = {'NPN', 'PNP', 'JFET_N', 'JFET_P', 'MFET_N_D', 'MFET_N_E', 'MFET_P_E', 'MFET_P_D', 'POT', }
    SPLIT_THICKNESS = 9

    for comp in components:
        comp_type = comp['component']
        cmp_box = np.asarray([comp['topleft']['x'], comp['topleft']['y'], comp['bottomright']['x'], comp['bottomright']['y']])  # [xmin, ymin, xmax, ymax]
        center = ((cmp_box[0:2] + cmp_box[2:4]) / 2).astype(np.int32)
        min_side_length = np.amin(cmp_box[2:4] - cmp_box[0:2])

        try:
            if comp_type in SPLIT_PERPENDICULAR:
                # draw a line through the component to split it
                dx = comp['pins'][1]['x'] - comp['pins'][0]['x']
                dy = comp['pins'][1]['y'] - comp['pins'][0]['y']
                pin_dir = np.array([dx, dy])
                normal_vec = np.array([-dy, dx])
                normal_vec = normal_vec / np.sqrt(np.sum(normal_vec**2))
                pin_a = np.array([comp['pins'][0]['x'], comp['pins'][0]['y']])

                cv.line(img, (pin_a + 0.4 * pin_dir - normal_vec * min_side_length * 0.6).astype(np.int32), (pin_a + 0.4 * pin_dir + normal_vec * min_side_length * 0.6).astype(np.int32), color=0, thickness=SPLIT_THICKNESS)
                cv.line(img, (pin_a + 0.6 * pin_dir - normal_vec * min_side_length * 0.6).astype(np.int32), (pin_a + 0.6 * pin_dir + normal_vec * min_side_length * 0.6).astype(np.int32), color=0, thickness=SPLIT_THICKNESS)
            elif comp_type == 'OPV':
                patch = img[cmp_box[1]:cmp_box[3]+1, cmp_box[0]:cmp_box[2]+1]

                # find contours in predicted patch
                contours, _ = cv.findContours((255 - patch * 255).astype(np.uint8), cv.RETR_LIST, cv.CHAIN_APPROX_TC89_KCOS)
                triangle = np.array([[0, 0], [2, 1], [0, 2]])
                best_matching_score = math.inf
                best_cnt = None
                for cnt in contours:
                    # search contour that closes resembles a triangle and is also near the patch center
                    convex = cv.convexHull(cnt)
                    r = cv.matchShapes(convex, triangle, cv.CONTOURS_MATCH_I3, 0.0)
                    m = cv.moments(cnt)
                    x = m['m10'] / m['m00']
                    y = m['m01'] / m['m00']
                    dist = math.sqrt((x + cmp_box[0] - center[0])**2 + (y + cmp_box[1] - center[1])**2)
                    if r * dist < best_matching_score:
                        best_matching_score = r * dist
                        best_cnt = convex

                # fill this contour, enlarge it and use as a mask to remove OPV from image
                fill_img = np.zeros_like(img, np.uint8)
                cv.drawContours(fill_img, [best_cnt + cmp_box[0:2]], 0, 255, cv.FILLED)
                fill_img = cv.dilate(fill_img, np.ones((25, 25)))
                img[fill_img > 128] = 0
            elif comp_type in SPLIT_BLOB:
                # draw a circle in the center of the object as a mask
                r = int(min_side_length / 2)
                cv.circle(img, center, r, 0, cv.FILLED)
            elif comp_type in {'MIC', 'SPK'}:
                pin_center = (np.array([comp['pins'][0]['x'] + comp['pins'][1]['x'], comp['pins'][0]['y'] + comp['pins'][1]['y']]) / 2).astype(np.int32)
                direction = np.abs(pin_center - center)
                if direction[0] > direction[1]:
                    # object is placed horizontally
                    cv.line(img, pin_center, (cmp_box[2], pin_center[1]), color=0, thickness=SPLIT_THICKNESS)
                    cv.line(img, (center[0], cmp_box[1]), (center[0], cmp_box[3]), color=0, thickness=SPLIT_THICKNESS)
                else:
                    cv.line(img, pin_center, (pin_center[0], cmp_box[3]), color=0, thickness=SPLIT_THICKNESS)
                    cv.line(img, (cmp_box[0], center[1]), (cmp_box[2], center[1]), color=0, thickness=SPLIT_THICKNESS)
            elif comp_type == 'S3':
                dir_a = np.array([comp['pins'][0]['x'] - center[0], comp['pins'][0]['y'] - center[1]])
                dir_b = np.array([comp['pins'][1]['x'] - center[0], comp['pins'][1]['y'] - center[1]])
                dir_c = np.array([comp['pins'][2]['x'] - center[0], comp['pins'][2]['y'] - center[1]])

                if dir_a @ dir_b > 0:
                    # dir_c is single
                    blob_center = (np.array([comp['pins'][0]['x'] + comp['pins'][1]['x'], comp['pins'][0]['y'] + comp['pins'][1]['y']]) / 2 * 0.7 + center * 0.3).astype(np.int32)
                elif dir_a @ dir_c > 0:
                    # dir_b is single
                    blob_center = (np.array([comp['pins'][0]['x'] + comp['pins'][2]['x'], comp['pins'][0]['y'] + comp['pins'][2]['y']]) / 2 * 0.7 + center * 0.3).astype(np.int32)
                else:
                    # dir_a is single
                    blob_center = (np.array([comp['pins'][2]['x'] + comp['pins'][1]['x'], comp['pins'][2]['y'] + comp['pins'][1]['y']]) / 2 * 0.7 + center * 0.3).astype(np.int32)

                r = int(min_side_length / 2)
                cv.circle(img, blob_center, r, 0, cv.FILLED)
        except Exception:
            continue

def prunePath(path, px_coor, corner_graph_indices):
    _, corners_on_path_i, _ = np.intersect1d(path, corner_graph_indices, return_indices=True)
    corners_on_path = path[np.sort(corners_on_path_i)]
    
    pruned_path = [px_coor[path[0]]]
    pruned_path.extend(px_coor[corners_on_path])
    pruned_path.append(px_coor[path[-1]])
    
    return np.asarray(pruned_path, np.int32)

def isCorner(a, b, c):
    v1 = a - b
    v2 = c - b
    cos_a = (v1 @ v2) / (np.sqrt(np.sum(v1**2)) * np.sqrt(np.sum(v2**2)))
    return abs(cos_a) < 0.85

def pruneGraph(pts, adjacency):
    for i in range(len(adjacency)):
        if np.count_nonzero(adjacency[i]) != 3:
            continue

        a = np.argmax(adjacency[i])
        c = len(adjacency) - np.argmax(adjacency[i][::-1]) - 1

        if not isCorner(pts[a], pts[i], pts[c]):
            adjacency[a, c] = 1
            adjacency[c, a] = 1
            adjacency[i, :] = 0
            adjacency[:, i] = 0

    return pts, adjacency

def buildNetGraphs(img, neuralOut):
    _, img = cv.threshold(img, 0, 1, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)

    MORPH_CLOSING_SIZE = 7
    img = cv.dilate(img, np.ones((MORPH_CLOSING_SIZE, MORPH_CLOSING_SIZE)))
    img = cv.erode(img, np.ones((MORPH_CLOSING_SIZE, MORPH_CLOSING_SIZE)))

    splitComponents(img, neuralOut)

    img = cv.resize(img, None, fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)
    scaleNeuralOutValues(neuralOut, 0.5)

    # skeletonize the image & produce adjacency matrix / graph
    skeleton_img = morphology.skeletonize(img)
    graph, px_coor = skan.csr.skeleton_to_csgraph(skeleton_img)
    px_coor = px_coor[..., ::-1]
    graph = scipy.sparse.csr_matrix(np.where(graph.toarray() > 0.5, 1.0, 0.0))

    # TODO: remove intersections that are not connected from graph

    skeleton_img = np.where(skeleton_img, 255, 0).astype(np.uint8)
    # cv.imshow('skeleton', skeleton_img)
    # cv.waitKey(0)

    # corner_dist = cv.cornerHarris(img, 5, 5, 0.05)
    # corners = np.stack(np.where(corner_dist > 0.1 * np.amax(corner_dist)), axis=-1)[:, ::-1]
    junctions = px_coor[np.where(np.sum(graph, axis=-1) > 2)[0]]
    corners = cv.goodFeaturesToTrack((img * 255).astype(np.uint8), 0, 0.2, 10)
    corners = [c.ravel() for c in corners]
    corners.extend(junctions)
    corners = np.asarray(corners, np.int32)

    # for junc in junctions:
    #     cv.circle(col_img, junc.astype(np.int32), 4, (0, 0, 255), -1)

    # for corn in corners:
    #     cv.circle(col_img, corn.astype(np.int32), 2, (0, 255, 0), -1)
    # cv.imshow('', col_img)
    # cv.waitKey()

    # linesP = cv.HoughLinesP(img, 1, np.pi / 180, threshold=10, minLineLength=0, maxLineGap=15)

    # lines = np.asarray([l[0] for l in linesP if ((l[0][0] - l[0][2])**2 + (l[0][1] - l[0][3])**2) > MIN_LINE_LENGTH**2])
    # angles = list(np.arctan2(lines[:, 3] - lines[:, 1], lines[:, 2] - lines[:, 0]))
    # lines = list(lines)

    # TODO: better starting point finding (currently the neural network prediction is directly used)
    cmp_pin_indices = []
    pin_points = []
    for c, cmp in enumerate(neuralOut):
        for p, pin in enumerate(cmp['pins']):
            # cmp_box = np.asarray([cmp['topleft']['x'], cmp['topleft']['y'], cmp['bottomright']['x'], cmp['bottomright']['y']])
            # _, start_point, _ = findPinStart(np.asarray([pin['x'], pin['y']]), cmp['component'], cmp_box, lines, angles)
            # cmp_pin_indices.append((c, p))
            # start_points.append(start_point)
            pin_points.append(np.asarray([pin['x'], pin['y']]))

    # find CSR matrix index for every start point (calculate distances for all combinations)
    pin_points = np.asarray(pin_points, np.int32)
    meshed_indices = np.stack(np.meshgrid(np.arange(px_coor.shape[0]), np.arange(pin_points.shape[0])), -1)
    dist = np.sum((pin_points[meshed_indices[:, :, 1]] - px_coor[meshed_indices[:, :, 0]])**2, axis=-1)
    pin_graph_indices = np.argmin(dist, -1)

    # find CSR matrix index for every corner point (calculate distances for all combinations)
    meshed_indices = np.stack(np.meshgrid(np.arange(px_coor.shape[0]), np.arange(corners.shape[0])), -1)
    dist = np.sum((corners[meshed_indices[:, :, 1]] - px_coor[meshed_indices[:, :, 0]])**2, axis=-1)
    corner_graph_indices = np.argmin(dist, -1)

    # for sp in px_coor[pin_graph_indices]:
    #     cv.circle(col_img, sp.astype(np.int32), 3, (255,0,0), -1)

    nx_graph = nx.to_networkx_graph(graph)

    net_list = []
    already_connected_indices = set()
    for i in range(len(pin_graph_indices)):
        if i in already_connected_indices:
            continue

        net_list.append([])

        for j in range(i+1, len(pin_graph_indices)):
            if j in already_connected_indices:
                continue

            try:
                path = np.asarray(nx.shortest_path(nx_graph, pin_graph_indices[i], pin_graph_indices[j]))
                path = prunePath(path, px_coor, corner_graph_indices)
                net_list[-1].append(path)

                # temporary drawing of path
                # cv.polylines(col_img, [path], False, (0,0,255), 2)

                already_connected_indices.add(i)
                already_connected_indices.add(j)
            except Exception:
                continue
        
        if not net_list[-1]:
            net_list[-1].append([px_coor[pin_graph_indices[i]]])

    net_graphs: List[nx.Graph] = []

    for net in net_list:
        points = np.concatenate(net)
        path_idcs = np.concatenate([[i] * len(net[i]) for i in range(len(net))])
        pt_idcs = np.concatenate([[j for j in range(len(net[i]))] for i in range(len(net))])

        # end_idcs = np.cumsum([len(p) for p in net]) - 1
        # start_idcs = np.concatenate(([0], np.cumsum([len(p) for p in net[:-1]], dtype=np.int32),))
        clusters = cluster.DBSCAN(eps=10, min_samples=1).fit(points)
        
        new_pts = [None] * (np.amax(clusters.labels_) + 1)
        adjacency_matrix = np.zeros((np.amax(clusters.labels_) + 1, np.amax(clusters.labels_) + 1))

        for l in range(np.amax(clusters.labels_) + 1):
            cluster_idcs = np.where(clusters.labels_ == l)[0]
            cluster_pts = points[cluster_idcs]
            avg_pt = np.sum(cluster_pts, 0) / len(cluster_pts)
            new_pts[l] = avg_pt

            adjacency_matrix[l, l] = 1

            for i in cluster_idcs:
                a = np.where(np.logical_and(pt_idcs == max(pt_idcs[i] - 1, 0), path_idcs == path_idcs[i]))[0][0]
                b = np.where(np.logical_and(pt_idcs == min(pt_idcs[i] + 1, len(net[path_idcs[i]]) - 1), path_idcs == path_idcs[i]))[0][0]
                a_label = clusters.labels_[a]
                b_label = clusters.labels_[b]
                adjacency_matrix[l, a_label] = 1
                adjacency_matrix[l, b_label] = 1

        new_pts, adjacency_matrix = pruneGraph(new_pts, adjacency_matrix)
        graph: nx.Graph = nx.from_numpy_array(adjacency_matrix)
        for n in list(graph):
            graph.nodes[n]['pos'] = new_pts[n]

        graph.remove_nodes_from(list(nx.isolates(graph)))
        net_graphs.append(graph)

    return net_graphs

def NetListExP(neuralOut, net_graphs):
    points = []
    net_ids = []
    for i, graph in enumerate(net_graphs):
        net_ids.extend([i] * len(list(graph.nodes)))
        for n in graph.nodes(data='pos'):
            points.append(n[1])
    points = np.array(points)
    NetList = []
    
    for comp in neuralOut:
        #gets topleft/bottomright cordinates
        topleft = [comp["topleft"]["x"], comp["topleft"]["y"]]
        botright = [comp["bottomright"]["x"], comp["bottomright"]["y"]]

        pins = []
        #iterates through the pins since their amount is variable
        for pincoll in comp["pins"]:
            x = pincoll["x"]
            y = pincoll["y"]
            dist = np.sum((points - [x, y])**2, axis=-1)
            nearest = np.argmin(dist)
            x = points[nearest][0]
            y = points[nearest][1]
            pins.append(
                {
                    "x": x, 
                    "y": y,
                    "id": net_ids[nearest]
                })
        #fill in the rest of the informations
        NetList.append({
            "component": comp["component"],
            "position": { 
                #center the component
                "x": topleft[0] + (abs(topleft[0] - botright[0]) / 2),
                "y": topleft[1] + (abs(topleft[1] - botright[1]) / 2)},
            "pins": pins
            })

    return NetList#reorderPins(NetList)

def lineList(neural_out, net_graphs):
    line_list = []
    for g in net_graphs:
        points = []
        successors = dict(nx.bfs_successors(g, 0))
        for n in g.nodes:
            if n not in successors:
                successors[n] = []
        succ_to_idx = {}
        for n in successors.keys():
            points.append({
                "pos": {
                    "x": g.nodes[n]['pos'][0],
                    "y": g.nodes[n]['pos'][1]
                },
                "connected": []
            })
            succ_to_idx[n] = len(points) - 1

        for n, succ in successors.items():
            points[succ_to_idx[n]]['connected'] = [succ_to_idx[s] for s in succ]

        net = {"points" : points}
        line_list.append(net)
    return line_list

def detect(neural_out, img):
    net_graphs = buildNetGraphs(img, neural_out)

    return NetListExP(neural_out, net_graphs), lineList(neural_out, net_graphs)

def main():
    neuralOutput = JsoncParser.parse_file('../DataProcessing/CompleteModel/TestData/test1.json')
    img = cv.imread("../DataProcessing/CompleteModel/TestData/test1.jpeg", cv.IMREAD_GRAYSCALE)
    
    #netList = NetListExP(neuralOutput)

    #netList = reorderPins(netList)

if __name__ == "__main__":
    main()