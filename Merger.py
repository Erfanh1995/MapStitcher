import geojson
import Graph
import math
import os
import time
import getopt
import sys
import networkx as nx
import pickle
import logging

interval = 5.0  # meters
matching_threshold = 15.0  # meters
bearing_limit = 45.0  # degrees

def vector(node1,node2):  # Find vector given two points
	return (node2[0] - node1[0], node2[1] - node1[1])

def length(v): # Find length given a vector
	return math.sqrt(v[0]**2 + v[1]**2)	

def path_bearing_meters(a_x, a_y, b_x, b_y):
    ydiff = float(b_y)-float(a_y)
    xdiff = float(b_x)-float(a_x) 
    bearing = math.atan2(ydiff, xdiff)
    return math.fmod(math.degrees(bearing) + 360.0, 360.0)

def bearing_difference(a_bearing, b_bearing):
    max_bearing = max(a_bearing, b_bearing)
    min_bearing = min(a_bearing, b_bearing)
    return min(abs(max_bearing - min_bearing), abs((360.0 - max_bearing) + min_bearing), abs(180.0 - (max_bearing - min_bearing)))

def open_mkdir(filename, *args, **kwargs):
    """
    Open a file, creating directories if necessary. Wraps the default python 'open' function
    """
    dirname = os.path.dirname(filename)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    return open(filename, *args, **kwargs)

def IsTheSameNode(node1,node2):
    if node1[0]==node2[0] and node1[1]==node2[1]:
        return True
    else:
        return False

def sum_tuple(tp1,tp2):
    return tuple(map(sum, zip(tp1, tp2)))

def avg_tuple(tp1,tp2):
    return tuple([(tp1[0]+tp2[0])/2,(tp1[1]+tp2[1])/2])


def check_sample_order(graph,edge,start):
    if length(vector(tuple(graph.nodes[start]),graph.samples[edge][0])) <= length(vector(tuple(graph.nodes[start]),graph.samples[edge][-1])):
        return True
    else:
        return False 

def matched_percentage(graph,edge,g_matched):
    matched = 0
    for i in graph.samples[edge]:
        if g_matched[i] != 0:
            matched +=1
    return (matched/len(graph.samples[edge]))*100

def merge_like_a_dream(graph1,graph2,matching_threshold,bearing_limit,filename,start_time):
    if os.path.exists(filename+"_matching.pickle"):
        with open_mkdir(filename+"_matching.pickle", 'rb') as handle:
            matching = pickle.load(handle)
    else:
        logging.info("-------------Starting the graph sampling-------------")
        print("-------------Starting the graph sampling-------------")
        graph = nx.Graph()
        graph.add_nodes_from(graph1.sampleHash.keys())
        graph.add_nodes_from(graph2.sampleHash.keys())
        flagcheck = False
        for node1 in graph1.sampleHash.keys():
            for node2 in graph2.sampleHash.keys():
                sample_distance = length(vector(node1,node2))
                if sample_distance <= matching_threshold:
                    for i in graph1.sampleHash[node1]:
                        for j in graph2.sampleHash[node2]:
                            bearing_i = path_bearing_meters(graph1.nodes[graph1.edges[i][0]][0], graph1.nodes[graph1.edges[i][0]][1], graph1.nodes[graph1.edges[i][1]][0], graph1.nodes[graph1.edges[i][1]][1])
                            bearing_j = path_bearing_meters(graph2.nodes[graph2.edges[j][0]][0], graph2.nodes[graph2.edges[j][0]][1], graph2.nodes[graph2.edges[j][1]][0], graph2.nodes[graph2.edges[j][1]][1])
                            difference = bearing_difference(bearing_i,bearing_j)
                            if difference < bearing_limit:
                                graph.add_edge(node1,node2, weight=(matching_threshold - sample_distance))
                                flagcheck = True
                                break
                        if flagcheck == True:
                            break
                flagcheck = False

        logging.info("-------------The graph is loaded - starting the matching algorithm-------------")
        print("-------------The graph is loaded - starting the matching algorithm-------------")
        matching = nx.algorithms.matching.max_weight_matching(graph)
        with open_mkdir(filename+"_matching.pickle", 'wb') as handle:
            pickle.dump(matching, handle, protocol=pickle.HIGHEST_PROTOCOL)

    logging.info("-------------Matching paths is done - writing files-------------")
    print("-------------Matching paths is done - writing files-------------")
    g2_matched = graph2.samples.copy()
    g2_matched_to = dict.fromkeys(graph2.sampleHash.keys(), 0)
    g1_matched_to = dict.fromkeys(graph1.sampleHash.keys(), 0)
    edge_matched_counter = dict.fromkeys(graph2.edges.keys(), 0)
    with open_mkdir(filename+"_matching.txt", 'w+') as eval_file:
        for edge in matching:
            sample_distance = length(vector(edge[0],edge[1]))
            if (edge[0] in graph1.sampleHash.keys()) and (edge[1] in graph2.sampleHash.keys()):
                for e in graph2.sampleHash[edge[1]]:
                    g2_matched[e] = [1 if x==edge[1] else x for x in g2_matched[e]]
                    edge_matched_counter[e] += 1
                g2_matched_to[edge[1]] = edge[0]
                g1_matched_to[edge[0]] = edge[1]
                eval_file.write(str(sample_distance) + "," + str(
                    edge[0][0]) + "," + str(
                    edge[0][1]) + "," + str(
                    edge[1][0]) + "," + str(
                    edge[1][1]) + "\n")
            elif (edge[1] in graph1.sampleHash.keys()) and (edge[0] in graph2.sampleHash.keys()):
                for e in graph2.sampleHash[edge[0]]:
                    g2_matched[e] = [1 if x==edge[0] else x for x in g2_matched[e]]
                    edge_matched_counter[e] += 1
                g2_matched_to[edge[0]] = edge[1]
                g1_matched_to[edge[1]] = edge[0]
                eval_file.write(str(sample_distance) + "," + str(
                    edge[1][0]) + "," + str(
                    edge[1][1]) + "," + str(
                    edge[0][0]) + "," + str(
                    edge[0][1]) + "\n")
            else:
                logging.warning("-------------WTH?-------------")

        for key in g1_matched_to:
            if g1_matched_to[key] == 0:
                eval_file.write("1000000" + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "\n")
        for key in g2_matched_to:
            if g2_matched_to[key] == 0:
                eval_file.write("2000000" + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "\n")

        eval_file.flush()
    logging.info("Time for weighted maximum matching of paths: " + str(time.time() - start_time))
    print("Time for weighted maximum matching of paths: " + str(time.time() - start_time))

    g2_matched_perc = dict.fromkeys(graph2.edges.keys(), 0)
    for e in graph2.edges.keys():
        if len(graph2.samples[e]) == 0:
            g2_matched_perc[e] = 0
        else:
            g2_matched_perc[e] = edge_matched_counter[e]/len(graph2.samples[e])

    # Matching intesections
    graph1_intersections = graph1.findIntersections(bearing_limit)
    graph2_intersections = graph2.findIntersections(bearing_limit)

    if os.path.exists(filename+"_intersections.pickle"):
        with open_mkdir(filename+"_intersections.pickle", 'rb') as handle:
            matching = pickle.load(handle)
    else:
        logging.info("-------------Starting the intesection matching-------------")
        print("-------------Starting the intesecttion matching-------------")
        graph = nx.Graph()
        graph.add_nodes_from(graph1_intersections.values())
        graph.add_nodes_from(graph2_intersections.values())
        for node1 in graph1_intersections.values():
            for node2 in graph2_intersections.values():
                sample_distance = length(vector(node1,node2))
                if sample_distance <= matching_threshold:
                    graph.add_edge(node1,node2, weight=(matching_threshold - sample_distance))


        logging.info("-------------The intersections are loaded - starting the intersection matching algorithm-------------")
        print("-------------The intersections are loaded - starting the intersection matching algorithm-------------")
        matching = nx.algorithms.matching.max_weight_matching(graph)
        with open_mkdir(filename+"_intersections.pickle", 'wb') as handle:
            pickle.dump(matching, handle, protocol=pickle.HIGHEST_PROTOCOL)

    logging.info("-------------Matching intersections is done - writing files-------------")
    print("-------------Matching intesections is done - writing files-------------")
    
    g2_in_matched_to = dict.fromkeys(graph2_intersections.values(), 0)
    g1_in_matched_to = dict.fromkeys(graph1_intersections.values(), 0)
    #edge_matched_counter = dict.fromkeys(graph2.edges.keys(), 0)
    with open_mkdir(filename+"_intersections.txt", 'w+') as eval_file:
        for edge in matching:
            sample_distance = length(vector(edge[0],edge[1]))
            if (edge[0] in graph1_intersections.values()) and (edge[1] in graph2_intersections.values()):
                    #edge_matched_counter[e] += 1
                g2_in_matched_to[edge[1]] = edge[0]
                g1_in_matched_to[edge[0]] = edge[1]
                eval_file.write(str(sample_distance) + "," + str(
                    edge[0][0]) + "," + str(
                    edge[0][1]) + "," + str(
                    edge[1][0]) + "," + str(
                    edge[1][1]) + "\n")
            elif (edge[1] in graph1_intersections.values()) and (edge[0] in graph2_intersections.values()):
                    #edge_matched_counter[e] += 1
                g2_in_matched_to[edge[0]] = edge[1]
                g1_in_matched_to[edge[1]] = edge[0]
                eval_file.write(str(sample_distance) + "," + str(
                    edge[1][0]) + "," + str(
                    edge[1][1]) + "," + str(
                    edge[0][0]) + "," + str(
                    edge[0][1]) + "\n")
            else:
                logging.warning("-------------WTH?-------------")

        for key in g1_in_matched_to:
            if g1_in_matched_to[key] == 0:
                eval_file.write("1000000" + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "\n")
        for key in g2_in_matched_to:
            if g2_in_matched_to[key] == 0:
                eval_file.write("2000000" + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "\n")

        eval_file.flush()

    logging.info("Time for weighted maximum matching of intersections: " + str(time.time() - start_time))
    print("Time for weighted maximum matching of intersections: " + str(time.time() - start_time))
    start_time = time.time()
    logging.info("-------------Starting the merge-------------")
    # Merging starts
    # Merge small portions missing
    """
    for i in g1_in_matched_to.keys():
        if g1_in_matched_to[i] == 0 and graph1.degree[graph1.nodeHash[i]] == 1: # intersection on g1 not matched to g2 and is degree-1
            edge = graph1.edgeLink[graph1.nodeHash[i]]
            if g1_in_matched_to[graph1.samples[edge][-1]] != 0 and matched_percentage(graph1,edge,g1_matched_to) > 9.0:
                if not check_sample_order(graph1,edge,i): # i is the end point and not the start point for the samples
                    v = vector(g1_in_matched_to[graph1.samples[edge][-1]],graph1.samples[edge][-1])
                else:
                    v = vector(g1_in_matched_to[graph1.samples[edge][0]],graph1.samples[edge][0])
    """
            
    # Merge big chunks from g2
    for intersection in g2_in_matched_to.keys():
        if g2_in_matched_to[intersection] == 0:
            closest_samples = []
            for node in graph2.nodeLink[graph2.nodeHash[intersection]]:
                edge = graph2.edgeHash[tuple([graph2.nodeHash[intersection],node])]
                if len(graph2.samples[edge]) == 0: # edge too short to have samples
                    #find the next edge to get the nearest sample
                    for next_node in graph2.nodeLink[node]:
                        #check that it has a similiar bearing. this is necessary for the average to be accurate
                        bearing_i = path_bearing_meters(graph2.nodes[graph2.edges[edge][0]][0], graph2.nodes[graph2.edges[edge][0]][1], graph2.nodes[graph2.edges[edge][1]][0], graph2.nodes[graph2.edges[edge][1]][1])
                        bearing_j = path_bearing_meters(graph2.nodes[graph2.edges[graph2.edgeHash[tuple([node,next_node])]][0]][0], graph2.nodes[graph2.edges[graph2.edgeHash[tuple([node,next_node])]][0]][1], graph2.nodes[graph2.edges[graph2.edgeHash[tuple([node,next_node])]][1]][0], graph2.nodes[graph2.edges[graph2.edgeHash[tuple([node,next_node])]][1]][1])
                        if next_node != graph2.nodeHash[intersection] and bearing_difference(bearing_i,bearing_j) <= 10: # this is the one
                            edge = graph2.edgeHash[tuple([node,next_node])]
                            break
                    if check_sample_order(graph2,edge,node):
                        if g2_matched_to[graph2.samples[edge][0]] != 0:
                            closest_samples.append(g2_matched_to[graph2.samples[edge][0]])
                    else:
                        if g2_matched_to[graph2.samples[edge][-1]] != 0:
                            closest_samples.append(g2_matched_to[graph2.samples[edge][-1]])
                    
                elif g2_matched_perc[edge] >= 0.9:
                    if check_sample_order(graph2,edge,graph2.nodeHash[intersection]):
                        if g2_matched_to[graph2.samples[edge][0]] != 0:
                            closest_samples.append(g2_matched_to[graph2.samples[edge][0]])
                    else:
                        if g2_matched_to[graph2.samples[edge][-1]] != 0:
                            closest_samples.append(g2_matched_to[graph2.samples[edge][-1]])
            if len(closest_samples) == 2: # we are only looking for a new intersection on the road (like converting a degree-2 vertex to degree-3 or 4 but in this case the vertex does not exist. it just happens to be in the middle of an edge)
                if graph1.sampleHash[closest_samples[0]] == graph1.sampleHash[closest_samples[1]]: # they both belong to the same edge
                    new_node = avg_tuple(closest_samples[0],closest_samples[1])
                    id = graph1.addNode(1,*new_node)
                    # cut the edge in half
                    old_edge = graph1.sampleHash[closest_samples[0]][0]
                    node1 = graph1.edges[old_edge][0]
                    node2 = graph1.edges[old_edge][1]
                    edge1 = graph1.connectTwoNodes(1,node1,id)
                    edge2 = graph1.connectTwoNodes(1,node2,id)
                    # split the samples to two sets and assign them to their new edges
                    split_index_1 = graph1.samples[old_edge].index(closest_samples[0])
                    split_index_2 = graph1.samples[old_edge].index(closest_samples[1])
                    if split_index_1 < split_index_2:
                        if length(vector(closest_samples[0],tuple(graph1.nodes[node1]))) < length(vector(closest_samples[1],tuple(graph1.nodes[node1]))):
                            graph1.samples[edge1] = graph1.samples[old_edge][0:split_index_1+1]
                            graph1.samples[edge2] = graph1.samples[old_edge][split_index_2:]
                        else: 
                            graph1.samples[edge1] = graph1.samples[old_edge][split_index_2:]
                            graph1.samples[edge2] = graph1.samples[old_edge][0:split_index_1+1]
                    else:
                        if length(vector(closest_samples[0],tuple(graph1.nodes[node1]))) < length(vector(closest_samples[1],tuple(graph1.nodes[node1]))):
                            graph1.samples[edge1] = graph1.samples[old_edge][0:split_index_2+1]
                            graph1.samples[edge2] = graph1.samples[old_edge][split_index_1:]
                        else:
                            graph1.samples[edge1] = graph1.samples[old_edge][split_index_1:]
                            graph1.samples[edge2] = graph1.samples[old_edge][0:split_index_2+1]
                    for sam in graph1.samples[edge1]:
                        graph1.sampleHash[sam] = edge1
                    for sam in graph1.samples[edge2]:
                        graph1.sampleHash[sam] = edge2
                    # remove the old edge
                    graph1.removeEdge(old_edge)
                else: # they belong to different edges
                    # so there is a degree-2 node between them that can be matched to the intersection on g2
                    candidates = {}
                    #edgeid = graph1.sampleHash[closest_samples[0]][0]
                    #print(edgeid)
                    #nodeids = graph1.edges[edgeid]
                    #node1id = nodeids[0]
                    #node2id = nodeids[1]
                    #node1 = tuple(graph1.nodes[node1id])
                    #node2 = tuple(graph1.nodes[node2id])
                    #candidates[node1] = length(vector(node1,intersection))
                    candidates[tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[0]][0]][0]])] = length(vector(tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[0]][0]][0]]),intersection))
                    candidates[tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[0]][0]][1]])] = length(vector(tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[0]][0]][1]]),intersection))
                    candidates[tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[1]][0]][0]])] = length(vector(tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[1]][0]][0]]),intersection))
                    candidates[tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[1]][0]][1]])] = length(vector(tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[1]][0]][1]]),intersection))
                    new_node = min(candidates, key=candidates.get)
                    if length(vector(new_node,intersection)) > matching_threshold:
                        print("Warning!")
                
                g2_in_matched_to[intersection] = new_node
                g1_in_matched_to[new_node] = intersection

                    
        if g2_in_matched_to[intersection] != 0:   # An intersection from g2 is matched to an intersection from g1
            #if graph1.degree[graph1.nodehash[i]] < graph2.degree[graph2.nodehash[g1_in_matched_to[i]]]: # The vertex is missing edges so let's add them
                # identify the edges that are not matched (the number might be more than the difference of vertex degrees)
            #v = vector(g2_in_matched_to[i],i)
            #for edge in graph2.edgeLink[graph2.nodehash[i]]:
            #    if g2_matched_perc[edge] < 0.1:
            stitch(graph1,graph2,g2_matched_to,g1_matched_to,g1_in_matched_to,g2_in_matched_to,g2_matched_perc,intersection)

                #if IsTheSameNode(graph2.nodes[graph2.edges[edge][0]],i): # first end-point is the intersection i
                #    if len(graph2.samples[edge]) >= 2:
                #        if g2_matched_to[graph2.samples[edge][0]] == 0 and g2_matched_to[graph2.samples[edge][1]] == 0: # AFAICT this edge is (most likely) not matched 
                            # start stitching till you reach a matched intersection or run out of connected new roads to add
                                 
                   # else:
                 #       if g2_matched_to[graph2.samples[edge][0]] == 0: # this edge is definitely not matched but the edge is super short so be careful here!
                 #           BFS(graph1,graph2,g2_matched_to,g1_matched_to,g1_in_matched_to,g2_in_matched_to,i) 


                #elif IsTheSameNode(graph2.nodes[graph2.edges[edge][1]],i): # second end-point is the intersection i
                #    if len(graph2.samples[edge]) >= 2:
                #        if g2_matched_to[graph2.samples[edge][len(graph2.samples[edge])-1]] == 0 and g2_matched_to[graph2.samples[edge][len(graph2.samples[edge])-2]] == 0: # AFAICT this edge is (most likely) not matched 
                            # start stitching
                #            BFS(graph1,graph2,g2_matched_to,g1_matched_to,g1_in_matched_to,g2_in_matched_to,i) 
                #    else:
                #        if g2_matched_to[graph2.samples[edge][len(graph2.samples[edge])-1]] == 0: # this edge is definitely not matched but be careful here!
                #            BFS(graph1,graph2,g2_matched_to,g1_matched_to,g1_in_matched_to,g2_in_matched_to,i) 

            #elif graph1.degree[graph1.nodehash[i]] == graph2.degree[graph2.nodehash[g1_in_matched_to[i]]]: # check to see if they have the same degree, their edges should all be matched with each other. (NO NEW EDGES)


        #else: # An intersection seems to be a road in the second map (possible degree-1 vertex)

"""
                else: # they belong to different edges
                    # so there is a degree-2 node between them that can be matched to the intersection on g2
                    candidates = {}
                    #edgeid = graph1.sampleHash[closest_samples[0]][0]
                    #print(edgeid)
                    #nodeids = graph1.edges[edgeid]
                    #node1id = nodeids[0]
                    #node2id = nodeids[1]
                    #node1 = tuple(graph1.nodes[node1id])
                    #node2 = tuple(graph1.nodes[node2id])
                    #candidates[node1] = length(vector(node1,intersection))
                    candidates[tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[0]][0]][0]])] = length(vector(tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[0]][0]][0]]),intersection))
                    candidates[tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[0]][0]][1]])] = length(vector(tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[0]][0]][1]]),intersection))
                    candidates[tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[1]][0]][0]])] = length(vector(tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[1]][0]][0]]),intersection))
                    candidates[tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[1]][0]][1]])] = length(vector(tuple(graph1.nodes[graph1.edges[graph1.sampleHash[closest_samples[1]][0]][1]]),intersection))
                    new_node = min(candidates, key=candidates.get)
                    if length(vector(candidates[new_node],intersection)) > matching_threshold:
                        print("Warning!")
"""                
             

"""
def BFS(graph1,graph2,g2_matched_to,g1_matched_to,g1_in_matched_to,g2_in_matched_to,g2_matched_perc,intersection):
    v = vector(g2_in_matched_to[intersection],intersection)
    queue = []
    visited = dict.fromkeys(g2_in_matched_to.keys(), False)
    queue.append(intersection)
    visited[intersection] = True
    while queue:
        s = queue.pop()
        for node in graph2.nodelink[graph2.nodeHash[s]]:
            if graph2.nodes[node] not in g2_in_matched_to.keys(): # The next node is not an intersection
                edge = graph2.edgeHash[graph2.nodeHash[s],node]
                if g2_matched_perc[edge] < 0.1:
                    # stitch the edge
                    matched_samples = stitch(graph1,graph2,g2_matched_to,g1_matched_to,edge,v,s)
                    # update g2_matched_perc
                    g2_matched_perc[edge] = (g2_matched_perc[edge]*len(graph2.samples[edge])+matched_samples)/len(graph2.samples[edge])
                    queue.append(node)

            elif g2_in_matched_to[tuple(graph2.nodes[node])] == 0 and visited[graph2.nodes[node]] == False: # The next node is an intersection but is not matched to an intersection in g1
                edge = graph2.edgeHash[graph2.nodeHash[s],node]
                if g2_matched_perc[edge] < 0.1:
                    # stitch the edge
                    matched_samples = stitch(graph1,graph2,g2_matched_to,g1_matched_to,edge,v,s)
                    # update g2_matched_perc
                    g2_matched_perc[edge] = (g2_matched_perc[edge]*len(graph2.samples[edge])+matched_samples)/len(graph2.samples[edge])
                    queue.append(node)
                visited[graph2.nodes[node]] = True
            else:
                s = queue.pop()
"""

def stitch(graph1,graph2,g2_matched_to,g1_matched_to,g1_in_matched_to,g2_in_matched_to,g2_matched_perc,intersection):
    queue = []
    prev_q = []
    visited = dict.fromkeys(graph2.nodes.keys(), False)
    visited_edge = dict.fromkeys(graph2.edges.keys(), False)
    queue.append(graph2.nodeHash[intersection]) # takes node id from graph2
    prev_q.append(graph1.nodeHash[g2_in_matched_to[intersection]]) # takes node id from graph1
    visited[intersection] = True
    #v = vector(g2_in_matched_to[intersection],intersection)
    while queue:
        s = queue.pop()
        prev = prev_q.pop()
        v = vector(tuple(graph2.nodes[s]),tuple(graph1.nodes[prev]))
        for node in graph2.nodeLink[s]:
            stitches = 0
            edge = graph2.edgeHash[tuple([s,node])]
            if visited[node] == False and len(graph2.samples[edge]) == 0:
                queue.append(node)
                prev_q.append(prev)
                visited[node] = True
                visited_edge[edge] = True
                continue

            if visited[node] == False and check_sample_order(graph2,edge,s) == False:
                        graph2.samples[edge].reverse()

            if tuple(graph2.nodes[node]) not in g2_in_matched_to.keys() and visited[node] == False: # The next node is not an intersection
                if g2_matched_perc[edge] < 0.1:
                    flag = False
                    for sample in graph2.samples[edge]:
                        if g2_matched_to[sample] == 0:
                            new_point = sum_tuple(sample,v)
                            id = graph1.addNode(1,*new_point)
                            graph1.connectTwoNodes(1,prev,id)
                            g2_matched_to[sample] = new_point
                            g1_matched_to[new_point] = sample
                            prev = id 
                            stitches += 1
                            flag = False
                        else:
                            # should we update v here? / does it become more accurate?
                            v = vector(sample,g2_matched_to[sample])
                            if flag == False:
                                id = graph1.addNode(1,*g2_matched_to[sample])
                                graph1.connectTwoNodes(1,prev,id)
                                flag = True
                                prev = id
                    
                    g2_matched_perc[edge] = (g2_matched_perc[edge]*len(graph2.samples[edge])+stitches)/len(graph2.samples[edge])
                    queue.append(node)
                    prev_q.append(prev)
                visited[node] = True
                visited_edge[edge] = True 
            
            elif tuple(graph2.nodes[node]) in g2_in_matched_to.keys() and g2_in_matched_to[tuple(graph2.nodes[node])] == 0 and visited[node] == False: # The next node is an intersection but is not matched to an intersection in g1
                if g2_matched_perc[edge] < 0.1:
                    flag = False
                    for sample in graph2.samples[edge]:
                        if g2_matched_to[sample] == 0:
                            new_point = sum_tuple(sample,v)
                            id = graph1.addNode(1,*new_point)
                            graph1.connectTwoNodes(1,prev,id)
                            g2_matched_to[sample] = new_point
                            g1_matched_to[new_point] = sample
                            prev = id 
                            stitches += 1
                            flag = False
                        else:
                            # should we update v here? / does it become more accurate? 
                            v = vector(sample,g2_matched_to[sample])
                            if flag == False:
                                id = graph1.addNode(1,*g2_matched_to[sample])
                                graph1.connectTwoNodes(1,prev,id) #this should only happen if the previous sample was not matched. if it was matched we should add a new edge
                                flag = True
                                prev = id
                    
                    g2_matched_perc[edge] = (g2_matched_perc[edge]*len(graph2.samples[edge])+stitches)/len(graph2.samples[edge])
                    # Add the intersection
                    new_intersection = sum_tuple(tuple(graph2.nodes[node]),v)
                    id = graph1.addNode(1,*new_intersection)
                    graph1.connectTwoNodes(1,prev,id)
                    g2_in_matched_to[tuple(graph2.nodes[node])] = new_intersection
                    g1_in_matched_to[new_intersection] = graph2.nodes[node]
                    prev = id 
                    queue.append(node)
                    prev_q.append(prev)
                visited[node] = True 
                visited_edge[edge] = True 

            elif tuple(graph2.nodes[node]) in g2_in_matched_to.keys() and g2_in_matched_to[tuple(graph2.nodes[node])] != 0 and visited[node] == False: #  The next node is an intersection and it is matched to an intersection in g1
                # Do we do anything here?
                if g2_matched_perc[edge] < 0.1:
                    flag = False
                    for sample in graph2.samples[edge]:
                        if g2_matched_to[sample] == 0:
                            new_point = sum_tuple(sample,v)
                            id = graph1.addNode(1,*new_point)
                            graph1.connectTwoNodes(1,prev,id)
                            g2_matched_to[sample] = new_point
                            g1_matched_to[new_point] = sample
                            prev = id 
                            stitches += 1
                            flag = False
                        else:
                            # should we update v here? / does it become more accurate?
                            v = vector(sample,g2_matched_to[sample])
                            if flag == False:
                                id = graph1.addNode(1,*g2_matched_to[sample])
                                graph1.connectTwoNodes(1,prev,id)
                                flag = True
                                prev = id
                    
                    g2_matched_perc[edge] = (g2_matched_perc[edge]*len(graph2.samples[edge])+stitches)/len(graph2.samples[edge])
                    # Add the intersection
                    new_intersection = sum_tuple(tuple(graph2.nodes[node]),v)
                    id = graph1.addNode(1,*new_intersection)
                    graph1.connectTwoNodes(1,prev,id)
                    g2_in_matched_to[tuple(graph2.nodes[node])] = new_intersection
                    g1_in_matched_to[new_intersection] = tuple(graph2.nodes[node])
                    # Here's where we prune the BFS tree and we dont add this node for further exploration
                visited[node] = True
                visited_edge[edge] = True

            # this might break the code
            elif visited[node] == True and visited_edge[edge] == False:
                # We still need to stitch this edge but we do not add this node for further exploration
                if g2_matched_perc[edge] < 0.1:
                    flag = False
                    for sample in graph2.samples[edge]:
                        if g2_matched_to[sample] == 0:
                            new_point = sum_tuple(sample,v)
                            id = graph1.addNode(1,*new_point)
                            graph1.connectTwoNodes(1,prev,id)
                            g2_matched_to[sample] = new_point
                            g1_matched_to[new_point] = sample
                            prev = id 
                            stitches += 1
                            flag = False
                        else:
                            # should we update v here? / does it become more accurate?
                            v = vector(sample,g2_matched_to[sample])
                            if flag == False:
                                id = graph1.addNode(1,*g2_matched_to[sample])
                                graph1.connectTwoNodes(1,prev,id)
                                flag = True
                                prev = id
                    
                    g2_matched_perc[edge] = (g2_matched_perc[edge]*len(graph2.samples[edge])+stitches)/len(graph2.samples[edge])
                visited_edge[edge] = True


"""
def old_stitch(graph1,graph2,g2_matched_to,g1_matched_to,g1_in_matched_to,g2_in_matched_to,edge,v,start):
    stitches = 0
    # check the order of the samples first. we don't know if they are ordered in the list the same way that we are going
    if check_sample_order(graph2,edge,start) == False:
        graph2.samples[edge].reverse()
    prev = graph1.nodeHash[g2_matched_to[start]] #TODO change! doesn't necessarily translate to to a node in g1
    for sample in graph2.samples[edge]:
        if g2_matched_to[sample] == 0:
            new_point = sum_tuple(sample,v)
            id = graph1.addNode(new_point)
            graph1.addEdge(prev,id)
            g2_matched_to[sample] = new_point
            g1_matched_to[new_point] = sample
            prev = id
            stitches +=1
        else:
            # should we update v here? / does it become more accurate?
            v = vector(sample,g2_matched_to[sample])
            graph1.addEdge(prev,graph1.nodeHash[g2_matched_to[sample]])
            prev = graph1.nodeHash[g2_matched_to[sample]]
    # identify the endpoint
    if IsTheSameNode(start,graph2.nodes[graph2.edges[edge][0]]):
        end = tuple(graph2.nodes[graph2.edges[edge][1]])
    elif IsTheSameNode(start,graph2.nodes[graph2.edges[edge][1]]):
        end = tuple(graph2.nodes[graph2.edges[edge][0]])
    # connect the last sample to the endpoint
    if end in g2_in_matched_to.keys(): # the endpoint is an intersection point in g2
        if g2_in_matched_to[end] == 0: # the endpoint is an intersection point in g2 but is not matched to any intersections in g1
            new_point = sum_tuple(end,v)
            id = graph1.addNode(new_point)
            graph1.addEdge(prev,id)
        else: # the endpoint is an intersection point in g2 and is matched to an intersections in g1
            point = graph1.nodeHash[g2_matched_to[end]]
            graph1.addEdge(prev,point)
    else: # the endpoint is just a vertex point in g2
        if end not in g2_matched_to.keys():
            new_point = sum_tuple(end,v)
            id = graph1.addNode(new_point)
            graph1.addEdge(prev,id)
        # if the endpoint happens to be a sample too it is already considered in samples and we don't care about it

    return stitches
"""

def weighted_max_matching(graph1,graph2,matching_threshold,bearing_limit,filename,start_time):
    if os.path.exists(filename+".pickle"):
        with open_mkdir(filename+".pickle", 'rb') as handle:
            matching = pickle.load(handle)
    else:
        logging.info("-------------Starting the graph sampling-------------")
        graph = nx.Graph()
        graph.add_nodes_from(graph1.breadcrumbHash.keys())
        graph.add_nodes_from(graph2.breadcrumbHash.keys())
        flagcheck = False
        for node1 in graph1.breadcrumbHash.keys():
            for node2 in graph2.breadcrumbHash.keys():
                breadcrumb_distance = length(vector(node1,node2))
                if breadcrumb_distance <= matching_threshold:
                    for i in graph1.breadcrumbHash[node1]:
                        for j in graph2.breadcrumbHash[node2]:
                            bearing_i = path_bearing_meters(graph1.nodes[graph1.edges[i][0]][0], graph1.nodes[graph1.edges[i][0]][1], graph1.nodes[graph1.edges[i][1]][0], graph1.nodes[graph1.edges[i][1]][1])
                            bearing_j = path_bearing_meters(graph2.nodes[graph2.edges[j][0]][0], graph2.nodes[graph2.edges[j][0]][1], graph2.nodes[graph2.edges[j][1]][0], graph2.nodes[graph2.edges[j][1]][1])
                            difference = bearing_difference(bearing_i,bearing_j)
                            if difference < bearing_limit:
                                graph.add_edge(node1,node2, weight=(matching_threshold - breadcrumb_distance))
                                flagcheck = True
                                break
                        if flagcheck == True:
                            break
                flagcheck = False

        logging.info("-------------The graph is loaded - starting the matching algorithm-------------")
        matching = nx.algorithms.matching.max_weight_matching(graph)
        with open_mkdir(filename+".pickle", 'wb') as handle:
            pickle.dump(matching, handle, protocol=pickle.HIGHEST_PROTOCOL)
    
    logging.info("-------------Matching is done - writing files-------------")
    g2_matched = graph2.breadcrumbs.copy()
    g2_matched_to = dict.fromkeys(graph2.breadcrumbHash.keys(), 0)
    g1_matched_to = dict.fromkeys(graph1.breadcrumbHash.keys(), 0)
    edge_matched_counter = dict.fromkeys(graph2.edges.keys(), 0)
    with open_mkdir(filename+".txt", 'w+') as eval_file:
        for edge in matching:
            breadcrumb_distance = length(vector(edge[0],edge[1]))
            if (edge[0] in graph1.breadcrumbHash.keys()) and (edge[1] in graph2.breadcrumbHash.keys()):
                for e in graph2.breadcrumbHash[edge[1]]:
                    g2_matched[e] = [1 if x==edge[1] else x for x in g2_matched[e]]
                    edge_matched_counter[e] += 1
                g2_matched_to[edge[1]] = edge[0]
                g1_matched_to[edge[0]] = edge[1]
                eval_file.write(str(breadcrumb_distance) + "," + str(
                    edge[0][0]) + "," + str(
                    edge[0][1]) + "," + str(
                    edge[1][0]) + "," + str(
                    edge[1][1]) + "\n")
            elif (edge[1] in graph1.breadcrumbHash.keys()) and (edge[0] in graph2.breadcrumbHash.keys()):
                for e in graph2.breadcrumbHash[edge[0]]:
                    g2_matched[e] = [1 if x==edge[0] else x for x in g2_matched[e]]
                    edge_matched_counter[e] += 1
                g2_matched_to[edge[0]] = edge[1]
                g1_matched_to[edge[1]] = edge[0]
                eval_file.write(str(breadcrumb_distance) + "," + str(
                    edge[1][0]) + "," + str(
                    edge[1][1]) + "," + str(
                    edge[0][0]) + "," + str(
                    edge[0][1]) + "\n")
            else:
                logging.warning("-------------WTH?-------------")

        for key in g1_matched_to:
            if g1_matched_to[key] == 0:
                eval_file.write("1000000" + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "\n")
        for key in g2_matched_to:
            if g2_matched_to[key] == 0:
                eval_file.write("2000000" + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "," + str(
                    key[0]) + "," + str(
                    key[1]) + "\n")

        eval_file.flush()
    logging.info("Time for weighted maximum matching: " + str(time.time() - start_time))
    start_time = time.time()
    logging.info("-------------Starting the merge-------------")
    length_of_matched = len(g2_matched)
    edges_to_remove = {}
    while length_of_matched != 0 or len(edge_matched_counter):
        for edge in edge_matched_counter.keys():
            if edge_matched_counter[edge] > 2:
            
                if g2_matched[edge][0] == 1 and g2_matched[edge][-1] == 1: # two endpoints are matched in addition to few error points in the middle (highly experimental. no solid logic here!)
                    if edge_matched_counter[edge]/len(g2_matched[edge]) < 0.75:
                        graph1.addEdge(graph2.edges[edge][0],g2_matched_to[graph2.breadcrumbs[edge][0]][0],g2_matched_to[graph2.breadcrumbs[edge][0]][1],graph2.edges[edge][1],g2_matched_to[graph2.breadcrumbs[edge][-1]][0],g2_matched_to[graph2.breadcrumbs[edge][-1]][1],edge)
                        logging.debug("Adding new edge 3: "+str(edge)+", "+ str(graph2.edges[edge][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][0]][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][0]][1])+", "+ str(graph2.edges[edge][1])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][-1]][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][-1]][1]))
                        for i in range(1,len(g2_matched[edge])-1):
                            if g2_matched[edge][i] != 1:
                                for e in graph2.breadcrumbHash[g2_matched[edge][i]]:
                                    if e != edge and e in g2_matched.keys():
                                        g2_matched[e] = [1 if x==g2_matched[edge][i] else x for x in g2_matched[e]]
                                        edge_matched_counter[e] += 1
                                g2_matched_to[g2_matched[edge][i]] = g2_matched[edge][i]
                                g2_matched[edge][i] = 1
                        del g2_matched[edge]
                        edges_to_remove[edge] = edge_matched_counter[edge]
                        length_of_matched -= 1

                elif g2_matched[edge][0] == 1: # partially but continously matched  
                    continuity_check = 0
                    for e in range(len(g2_matched[edge])):
                        if continuity_check == 2:
                            break
                        if g2_matched[edge][e] != 1:
                            continuity_check +=1
                            breakpoint = e-1
                    if continuity_check == 1:
                        graph1.addEdge(graph2.edges[edge][0],g2_matched_to[graph2.breadcrumbs[edge][breakpoint]][0],g2_matched_to[graph2.breadcrumbs[edge][breakpoint]][1],graph2.edges[edge][1],graph2.nodes[graph2.edges[edge][1]][0],graph2.nodes[graph2.edges[edge][1]][1],edge)
                        logging.debug("Adding new edge 3.1: "+ str(edge)+", "+ str(graph2.edges[edge][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][breakpoint]][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][breakpoint]][1])+", "+ str(graph2.edges[edge][1])+", "+ str(graph2.nodes[graph2.edges[edge][1]][0])+", "+ str(graph2.nodes[graph2.edges[edge][1]][1]))
                        for i in range(breakpoint+1,len(g2_matched[edge])):
                            for e in graph2.breadcrumbHash[g2_matched[edge][i]]:
                                if e != edge and e in g2_matched.keys():
                                    g2_matched[e] = [1 if x==g2_matched[edge][i] else x for x in g2_matched[e]]
                                    edge_matched_counter[e] += 1
                            g2_matched_to[g2_matched[edge][i]] = g2_matched[edge][i] 
                            g2_matched[edge][i] = 1 
                        del g2_matched[edge]
                        edges_to_remove[edge] = edge_matched_counter[edge]
                        length_of_matched -= 1


                elif g2_matched[edge][-1] == 1: # partially but continously matched  
                    continuity_check = 0
                    for e in range(len(g2_matched[edge]) - 1, -1, -1):
                        if continuity_check == 2:
                            break
                        if g2_matched[edge][e] != 1:
                            continuity_check +=1
                            breakpoint = e-1
                    if continuity_check == 1:
                        graph1.addEdge(graph2.edges[edge][0],graph2.nodes[graph2.edges[edge][0]][0],graph2.nodes[graph2.edges[edge][0]][1],graph2.edges[edge][1],g2_matched_to[graph2.breadcrumbs[edge][breakpoint]][0],g2_matched_to[graph2.breadcrumbs[edge][breakpoint]][1],edge)
                        logging.debug("Adding new edge 3.2: "+str(edge)+", "+ str(graph2.edges[edge][0])+", "+ str(graph2.nodes[graph2.edges[edge][0]][0])+", "+ str(graph2.nodes[graph2.edges[edge][0]][1])+", "+ str(graph2.edges[edge][1])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][breakpoint]][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][breakpoint]][1]))
                        for i in range(0, breakpoint+1):
                            for e in graph2.breadcrumbHash[g2_matched[edge][i]]:
                                if e != edge and e in g2_matched.keys():
                                    g2_matched[e] = [1 if x==g2_matched[edge][i] else x for x in g2_matched[e]]
                                    edge_matched_counter[e] += 1
                            g2_matched_to[g2_matched[edge][i]] = g2_matched[edge][i]
                            g2_matched[edge][i] = 1 
                        del g2_matched[edge]
                        edges_to_remove[edge] = edge_matched_counter[edge]
                        length_of_matched -= 1
                        
            elif edge_matched_counter[edge] == 2: # only two endpoints are matched (NOT the edge has more breadcrumbs than just the endpoints)
                if g2_matched[edge][0] == 1 and g2_matched[edge][-1] == 1:
                    graph1.addEdge(graph2.edges[edge][0],g2_matched_to[graph2.breadcrumbs[edge][0]][0],g2_matched_to[graph2.breadcrumbs[edge][0]][1],graph2.edges[edge][1],g2_matched_to[graph2.breadcrumbs[edge][-1]][0],g2_matched_to[graph2.breadcrumbs[edge][-1]][1],edge)
                    logging.debug("Adding new edge 2: "+str(edge)+", "+ str(graph2.edges[edge][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][0]][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][0]][1])+", "+ str(graph2.edges[edge][1])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][-1]][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][-1]][1]))
                    for i in range(1,len(g2_matched[edge])-1):
                        for e in graph2.breadcrumbHash[g2_matched[edge][i]]:
                            if e != edge and e in g2_matched.keys():
                                g2_matched[e] = [1 if x==g2_matched[edge][i] else x for x in g2_matched[e]]
                                edge_matched_counter[e] += 1
                        g2_matched_to[g2_matched[edge][i]] = g2_matched[edge][i]
                        g2_matched[edge][i] = 1
                    del g2_matched[edge]
                    edges_to_remove[edge] = edge_matched_counter[edge]
                    length_of_matched -= 1
                    
            elif edge_matched_counter[edge] == 1: # only the first endpoint is matched
                if g2_matched[edge][0] == 1:
                    #print(g2_matched_to[graph2.breadcrumbs[edge][0]])
                    graph1.addEdge(graph2.edges[edge][0],g2_matched_to[graph2.breadcrumbs[edge][0]][0],g2_matched_to[graph2.breadcrumbs[edge][0]][1],graph2.edges[edge][1],graph2.nodes[graph2.edges[edge][1]][0],graph2.nodes[graph2.edges[edge][1]][1],edge)
                    logging.debug("Adding new edge 1.1: "+ str(edge)+", "+ str(graph2.edges[edge][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][0]][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][0]][1])+", "+ str(graph2.edges[edge][1])+", "+ str(graph2.nodes[graph2.edges[edge][1]][0])+", "+ str(graph2.nodes[graph2.edges[edge][1]][1]))
                    for i in range(1,len(g2_matched[edge])):
                        for e in graph2.breadcrumbHash[g2_matched[edge][i]]:
                            if e != edge and e in g2_matched.keys():
                                g2_matched[e] = [1 if x==g2_matched[edge][i] else x for x in g2_matched[e]]
                                edge_matched_counter[e] += 1
                        g2_matched_to[g2_matched[edge][i]] = g2_matched[edge][i] 
                        g2_matched[edge][i] = 1 
                    del g2_matched[edge]
                    edges_to_remove[edge] = edge_matched_counter[edge]
                    length_of_matched -= 1

                elif g2_matched[edge][-1] == 1: # only the last endpoint is matched
                    #print(g2_matched_to[graph2.breadcrumbs[edge][-1]])
                    graph1.addEdge(graph2.edges[edge][0],graph2.nodes[graph2.edges[edge][0]][0],graph2.nodes[graph2.edges[edge][0]][1],graph2.edges[edge][1],g2_matched_to[graph2.breadcrumbs[edge][-1]][0],g2_matched_to[graph2.breadcrumbs[edge][-1]][1],edge)
                    logging.debug("Adding new edge 1.2: "+str(edge)+", "+ str(graph2.edges[edge][0])+", "+ str(graph2.nodes[graph2.edges[edge][0]][0])+", "+ str(graph2.nodes[graph2.edges[edge][0]][1])+", "+ str(graph2.edges[edge][1])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][-1]][0])+", "+ str(g2_matched_to[graph2.breadcrumbs[edge][-1]][1]))
                    for i in range(0,len(g2_matched[edge])-1):
                        for e in graph2.breadcrumbHash[g2_matched[edge][i]]:
                            if e != edge and e in g2_matched.keys():
                                g2_matched[e] = [1 if x==g2_matched[edge][i] else x for x in g2_matched[e]]
                                edge_matched_counter[e] += 1
                        g2_matched_to[g2_matched[edge][i]] = g2_matched[edge][i]
                        g2_matched[edge][i] = 1  
              
                    del g2_matched[edge]
                    edges_to_remove[edge] = edge_matched_counter[edge]
                    length_of_matched -= 1

        if len(edges_to_remove) == 0:  # nothing else is going to be added - BREAK
            #for edge in edge_matched_counter.keys():
            #    if edge_matched_counter[edge] == 0:
            #         graph1.addEdge(graph2.edges[edge][0],graph2.nodes[graph2.edges[edge][0]][0],graph2.nodes[graph2.edges[edge][0]][1],graph2.edges[edge][1],graph2.nodes[graph2.edges[edge][1]][0],graph2.nodes[graph2.edges[edge][1]][1],edge)
            break
        edge_matched_counter = {k: v for k, v in edge_matched_counter.items() if k not in edges_to_remove}
        edges_to_remove.clear()

    logging.info("Time for merge: " + str(time.time() - start_time))

if __name__ == '__main__':
    help_string = ("Usage: python graphsampling.py <data_folder> <first map> <second map> "
                   "[-i <interval>] "
                   "[-b <bearing_limit>] "
                   "[-t <match_distance_threshold>] "
                   "[-h]\n")
    if len(sys.argv) < 4:
        print(help_string)
        exit()
    (opts, args) = getopt.getopt(sys.argv[5:], "i:b:t:h")

    for o, a in opts:
        if o == "-i":
            interval = float(a)
        if o == "-b":
            bearing_limit = float(a)
        if o == "-t":
            matching_threshold = float(a)
        if o == "-h":
            print(help_string)
            exit()

    start_time = time.time()
    data_path = sys.argv[1].strip("/\\")
    dataset = sys.argv[2]
    map1 = sys.argv[3]
    map2 = sys.argv[4]
    f_data = "{folder}/{dataset}".format(folder=data_path, dataset=dataset)
    f_map1 = "{folder}/{map}/{dataset}_{map}".format(folder=f_data, dataset=dataset, map=map1)
    f_map2 = "{folder}/{map}/{dataset}_{map}".format(folder=f_data, dataset=dataset, map=map2)
    f_results = "{folder}/evals/{dataset}_{map1}_{map2}_wmm{matchingdist}".format(folder=f_data, dataset=dataset, map1=map1, map2=map2, matchingdist=str(matching_threshold))
    logging.basicConfig(filename=f_results+"_debug.log", level=logging.DEBUG, format="%(asctime)s %(message)s", filemode="w")
    logging.debug("Debug logging test...")
    print("-------------Reading graphs-------------")
    logging.info("-------------Reading graphs-------------")
    if os.path.exists(f_map1+"_graph.pickle"):
        logging.info("Time for reading map 1 from a saved file "+ str(time.time() - start_time))
        print("Time for reading map 1 from a saved file  "+ str(time.time() - start_time))
        with open_mkdir(f_map1+"_graph.pickle", 'rb') as handle:
            graph1 = pickle.load(handle)
    else:
        start_time = time.time()
        graph1 = Graph.Graph(f_map1)
        logging.info("Time for reading map 1: "+ str(time.time() - start_time))
        print("Time for reading map 1: "+ str(time.time() - start_time))
        start_time = time.time()
        print("-------------Putting Samples-------------")
        logging.info("-------------Putting Samples-------------")
        graph1.putSamples(interval)
        logging.info("Time for putting samples: "+ str(time.time() - start_time))
        print("Time for putting samples: "+ str(time.time() - start_time))
        with open_mkdir(f_map1+"_graph.pickle", 'wb') as handle:
                pickle.dump(graph1, handle, protocol=pickle.HIGHEST_PROTOCOL)

    if os.path.exists(f_map2+"_graph.pickle"):
        logging.info("Time for reading map 2 from a saved file "+ str(time.time() - start_time))
        print("Time for reading map 2 from a saved file  "+ str(time.time() - start_time))
        with open_mkdir(f_map2+"_graph.pickle", 'rb') as handle:
            graph2 = pickle.load(handle)
    else:
        start_time = time.time()
        graph2 = Graph.Graph(f_map2)
        logging.info("Time for reading map 2: "+ str(time.time() - start_time))
        print("Time for reading map 2: "+ str(time.time() - start_time))
        start_time = time.time()
        print("-------------Putting Samples-------------")
        logging.info("-------------Putting Samples-------------")
        graph2.putSamples(interval)
        logging.info("Time for putting samples: "+ str(time.time() - start_time))
        print("Time for putting samples: "+ str(time.time() - start_time))
        with open_mkdir(f_map2+"_graph.pickle", 'wb') as handle:
                pickle.dump(graph2, handle, protocol=pickle.HIGHEST_PROTOCOL)
    start_time = time.time()
    #weighted_max_matching(graph1,graph2,matching_threshold,bearing_limit,f_results,start_time)
    merge_like_a_dream(graph1,graph2,matching_threshold,bearing_limit,f_results,start_time)
    f_newmap = "{folder}/{dataset}_{map}".format(folder=f_data,dataset=dataset,map=map1+"_"+map2)
    graph1.Dump2txt(f_newmap)