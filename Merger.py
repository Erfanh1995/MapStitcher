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
    for i in g1_in_matched_to.keys():
        if g1_in_matched_to[i] != 0:   # An intersection from g1 is matched to an intersection from g2
            if graph1.degree[graph1.nodehash[i]] < graph2.degree[graph2.nodehash[g1_in_matched_to[i]]]: # The vertex is missing edges so let's add them
                # identify the edges that are not matched (the number might be more than the difference of vertex degrees)
                for edge in graph1.edgeLink[i]:
                    if graph1.nodes[graph1.edges[edge][0]] == i: #first point is the intersection i

                    elif graph1.nodes[graph1.edges[edge][1]] == i: #second point is the intersection i



            elif graph1.degree[graph1.nodehash[i]] == graph2.degree[graph2.nodehash[g1_in_matched_to[i]]]: # check to see if they have the same degree, their edges should all be matched with each other. (NO NEW EDGES)


        else: # An intersection seems to be a road in the second map (possible degree-1 vertex)

                









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
    graph1 = Graph.Graph(f_map1)
    graph2 = Graph.Graph(f_map2)
    logging.info("Time for reading graphs: "+ str(time.time() - start_time))
    print("Time for reading graphs: "+ str(time.time() - start_time))
    start_time = time.time()
    print("-------------Putting Samples-------------")
    logging.info("-------------Putting Samples-------------")
    graph1.putSamples(interval)
    graph2.putSamples(interval)
    logging.info("Time for putting samples: "+ str(time.time() - start_time))
    print("Time for putting samples: "+ str(time.time() - start_time))
    #start_time = time.time()
    #weighted_max_matching(graph1,graph2,matching_threshold,bearing_limit,f_results,start_time)
    merge_like_a_dream(graph1,graph2,matching_threshold,bearing_limit,f_results,start_time)
    #f_newmap = "{folder}/{dataset}_{map}".format(folder=f_data,dataset=dataset,map=map1+"_"+map2)
    #graph1.Dump2txt(f_newmap)