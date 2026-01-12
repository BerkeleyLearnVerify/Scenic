import sys
import math
import numpy as np
import traceback

from scenic.formats.opendrive import xodr_parser
from shapely.geometry import Polygon, Point, LineString
from scenic.core.geometry import cleanPolygon, cleanChain, averageVectors
from scenic.core.regions import PolylineRegion, PolygonalRegion


def create_center_line(leftEdge, rightEdge):
        # Heuristically create some kind of reasonable centerline
        leftPoints = list(leftEdge.lineString.coords)
        rightPoints = list(rightEdge.lineString.coords)

        if len(leftPoints) == len(rightPoints):
            centerPoints = list(
                averageVectors(l, r) for l, r in zip(leftPoints, rightPoints)
            )
        else:
            num = max(len(leftPoints), len(rightPoints))
            centerPoints = []
            for d in np.linspace(0, 1, num):
                l = leftEdge.lineString.interpolate(d, normalized=True)
                r = rightEdge.lineString.interpolate(d, normalized=True)
                centerPoints.append(averageVectors(l.coords[0], r.coords[0]))
        centerline = PolylineRegion(cleanChain(centerPoints))

        return centerline

def construct_crosswalk_polys(points, tolerance):
    crosswalk_polygon = cleanPolygon(Polygon(points), tolerance)
    crosswalk_polygon_coords = list(crosswalk_polygon.exterior.coords[:-1])

    mrr = list(crosswalk_polygon.minimum_rotated_rectangle.exterior.coords)[:-1] #Drop last point - same as the first point
    edges = []
    lengths = []
    
    for i in range(4):
        a = mrr[i]
        b = mrr[(i + 1) % 4] #Wrap around to the first point

        edge = (a,b) #Form the edge between the two points: a and b
        edges.append(edge)
    
    for (a,b) in edges:
        dx = b[0] - a[0] #Difference in x-coordinates
        dy = b[1] - a[1] #Difference in y-coordinates

        distance = math.hypot(dx, dy) #Calculate the distance between the two points
        lengths.append(distance)
    
    get_mrr_shortest_indices = np.argsort(lengths)[:2] #Get the fist two indices - they correspond to the shortest edges

    mrr_shortest_edge = edges[get_mrr_shortest_indices[0]]
    mrr_shortest_opp_edge = edges[get_mrr_shortest_indices[1]]

    def get_edge_midpoint(vertex1, vertex2):
        return (np.asarray(vertex1) + np.asarray(vertex2)) / 2
    
    m1 = get_edge_midpoint(mrr_shortest_edge[0], mrr_shortest_edge[1])
    m2 = get_edge_midpoint(mrr_shortest_opp_edge[0], mrr_shortest_opp_edge[1])

    mrr_shortest_edge_distances = []
    mrr_shortest_opp_edge_distances = []

    mrr_shortest_edge = LineString(mrr_shortest_edge)
    mrr_shortest_opp_edge = LineString(mrr_shortest_opp_edge)

    for x,y in crosswalk_polygon_coords:
        crosswalk_polygon_vertex = Point(x,y)
        mrr_shortest_opp_edge_distances.append(crosswalk_polygon_vertex.distance(mrr_shortest_opp_edge)) #Distance from each vertex on the crosswalk polygon to the bottom edge of the MRR (shortest distance to any point on the bottom edge)
        mrr_shortest_edge_distances.append(crosswalk_polygon_vertex.distance(mrr_shortest_edge))

    bottom_cut_index = 0
    top_cut_index = 0
    final_bottom_score = float("inf")
    final_top_score = float("inf")

    for i in range(len(crosswalk_polygon_coords)):
        j = (i + 1) % len(crosswalk_polygon_coords)
        
        b_score = max(mrr_shortest_opp_edge_distances[i], mrr_shortest_opp_edge_distances[j]) #Max of the two distances to the MRR bottom edge
        if b_score < final_bottom_score:
            final_bottom_score = b_score
            bottom_cut_index = i
        
        t_score = max(mrr_shortest_edge_distances[i], mrr_shortest_edge_distances[j]) #Max of the two distances to the MRR top edge
        if t_score < final_top_score:
            final_top_score = t_score
            top_cut_index = i

    top_cut = (top_cut_index, (top_cut_index + 1) % len(crosswalk_polygon_coords)) #Indices of the two vertices that define the top cut
    top_cut_vertices = (crosswalk_polygon_coords[top_cut[0]], crosswalk_polygon_coords[top_cut[1]])
    bottom_cut = (bottom_cut_index, (bottom_cut_index + 1) % len(crosswalk_polygon_coords)) #Indices of the two vertices that define the bottom cut
    bottom_cut_vertices = (crosswalk_polygon_coords[bottom_cut[0]], crosswalk_polygon_coords[bottom_cut[1]])

    def walk_polygon(n, bottom_cut, top_cut):
        t0, t1 = top_cut #top cut edge indices (t0 -> t1)
        b0, b1 = bottom_cut #bottom edge indices (b0 -> b1)

        left_indices = [t0]
        i = t0
        while i != b1:
            i = (i - 1 + n) % n
            left_indices.append(i)
        
        right_indices = [t1]
        i = t1
        while i != b0:
            i = (i + 1) % n
            right_indices.append(i)
        
        return left_indices, right_indices
    
    left_indices, right_indices = walk_polygon(len(crosswalk_polygon_coords), bottom_cut, top_cut)
    left_edge = [crosswalk_polygon_coords[i] for i in left_indices]
    right_edge = [crosswalk_polygon_coords[i] for i in right_indices]

    # print(f"Top edge: {top_cut_vertices}. Left edge: {left_edge}")
    # print(f"Bottom edge: {bottom_cut_vertices}. Right edge: {right_edge}") 

    leftEdge = PolylineRegion(cleanChain(left_edge))
    rightEdge = PolylineRegion(cleanChain(right_edge))
    centerLine = create_center_line(leftEdge, rightEdge)
    # for point in centerLine.lineString.coords:
    #     print(f"Center line point: {point}")


    return crosswalk_polygon, centerLine, leftEdge, rightEdge

def lots_of_vertices():
    return [(-10, 11), (-4, 13), (2, 12), (7, 11), (9, 5), (8, -1), (2, -3), (-6, -2), (-11, 3)]
def l_shaped():
    return [(5,-3), (3,5), (0,4), (2,-5), (13,-5), (11,-3)]

        
if __name__ == "__main__":
    #shape1 = lots_of_vertices()
    shape2 = l_shaped()
    n = len(shape2)

    for k in range(n):
        rotated = shape2[k:] + shape2[:k]
        #print(f"Rotated: {rotated}")
        crosswalk_polygon, centerLine, leftEdge, rightEdge = construct_crosswalk_polys(rotated, 0.05)
        
    #crosswalk_polygon, centerLine, leftEdge, rightEdge = construct_crosswalk_polys(shape2, 0.05)
