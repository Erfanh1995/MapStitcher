import Graph
import random

def split(graph):
    g1 = Graph.Graph()
    g2 = Graph.Graph()
    list_of_edges = list(graph.edges.keys())
    id1 = random.randint(0,len(list_of_edges)-1)
    id2 = random.randint(0,len(list_of_edges)-1)
    int_edge1 = graph.edges[list_of_edges[id1]]
    int_edge2 = graph.edges[list_of_edges[id2]]
    if int_edge1 == int_edge2:
        id2 = random.randint(0,len(list_of_edges)-1)
        int_edge2 = graph.edges[id2]
    g1.addNode(int_edge1[0],graph.nodes[int_edge1[0]][0],graph.nodes[int_edge1[0]][1])
    g1.addNode(int_edge1[1],graph.nodes[int_edge1[1]][0],graph.nodes[int_edge1[1]][1])
    g1.connectTwoNodes(id1,int_edge1[0],int_edge1[1])
    g2.addNode(int_edge2[0],graph.nodes[int_edge2[0]][0],graph.nodes[int_edge2[0]][1])
    g2.addNode(int_edge2[1],graph.nodes[int_edge2[1]][0],graph.nodes[int_edge2[1]][1])
    g2.connectTwoNodes(id2,int_edge2[0],int_edge2[1])
    del list_of_edges[id1]
    del list_of_edges[id2]
    candidates1 = []
    candidates2 = []
    candidates1.extend(graph.edgeLink[int_edge1[0]])
    candidates1.extend(graph.edgeLink[int_edge1[1]])
    candidates2.extend(graph.edgeLink[int_edge2[0]])
    candidates2.extend(graph.edgeLink[int_edge2[1]])
    while len(list_of_edges)>0 and len(candidates1)>0 and len(candidates2)>0:
        print(len(list_of_edges), len(candidates1), len(candidates2))
        if len(candidates1) > 1:
            cand1 = random.randint(0,len(candidates1)-1)
        else:
            cand1 = 0
        if len(candidates2) > 1:
            cand2 = random.randint(0,len(candidates2)-1)
        else:
            cand2 = 0

        if candidates1[cand1] in candidates2:
            cand2 = candidates2.index(candidates1[cand1])
        elif candidates2[cand2] in candidates1:
            cand1 = candidates1.index(candidates2[cand2])
        
        if graph.edges[candidates1[cand1]][0] in g1.nodes.keys() and graph.edges[candidates1[cand1]][1] in g1.nodes.keys():
            g1.connectTwoNodes(candidates1[cand1],graph.edges[candidates1[cand1]][0],graph.edges[candidates1[cand1]][1])
        elif graph.edges[candidates1[cand1]][0] in g1.nodes.keys():
            g1.addNode(graph.edges[candidates1[cand1]][1],graph.nodes[graph.edges[candidates1[cand1]][1]][0],graph.nodes[graph.edges[candidates1[cand1]][1]][1])
            for edge in graph.edgeLink[graph.edges[candidates1[cand1]][1]]:
                if (edge in list_of_edges) and (edge not in candidates1):
                    candidates1.append(edge)
            g1.connectTwoNodes(candidates1[cand1],graph.edges[candidates1[cand1]][0],graph.edges[candidates1[cand1]][1])
        elif graph.edges[candidates1[cand1]][1] in g1.nodes.keys():
            g1.addNode(graph.edges[candidates1[cand1]][0],graph.nodes[graph.edges[candidates1[cand1]][0]][0],graph.nodes[graph.edges[candidates1[cand1]][0]][1])
            for edge in graph.edgeLink[graph.edges[candidates1[cand1]][0]]:
                if (edge in list_of_edges) and (edge not in candidates1):
                    candidates1.append(edge)
            g1.connectTwoNodes(candidates1[cand1],graph.edges[candidates1[cand1]][0],graph.edges[candidates1[cand1]][1])
        else:
            print("Warning")

        if graph.edges[candidates2[cand2]][0] in g2.nodes.keys() and graph.edges[candidates2[cand2]][1] in g2.nodes.keys():
            g2.connectTwoNodes(candidates2[cand2],graph.edges[candidates2[cand2]][0],graph.edges[candidates2[cand2]][1])
        elif graph.edges[candidates2[cand2]][0] in g2.nodes.keys():
            g2.addNode(graph.edges[candidates2[cand2]][1],graph.nodes[graph.edges[candidates2[cand2]][1]][0],graph.nodes[graph.edges[candidates2[cand2]][1]][1])
            for edge in graph.edgeLink[graph.edges[candidates2[cand2]][1]]:
                if (edge in list_of_edges) and (edge not in candidates2):
                    candidates2.append(edge)
            g2.connectTwoNodes(candidates2[cand2],graph.edges[candidates2[cand2]][0],graph.edges[candidates2[cand2]][1])
        elif graph.edges[candidates2[cand2]][1] in g2.nodes.keys():
            g2.addNode(graph.edges[candidates2[cand2]][0],graph.nodes[graph.edges[candidates2[cand2]][0]][0],graph.nodes[graph.edges[candidates2[cand2]][0]][1])
            for edge in graph.edgeLink[graph.edges[candidates2[cand2]][0]]:
                if (edge in list_of_edges) and (edge not in candidates2):
                    candidates2.append(edge)
            g2.connectTwoNodes(candidates2[cand2],graph.edges[candidates2[cand2]][0],graph.edges[candidates2[cand2]][1])
        else:
            print("Warning")
        
        if candidates1[cand1] in list_of_edges:
            list_of_edges.remove(candidates1[cand1])
        if candidates2[cand2] in list_of_edges:
            list_of_edges.remove(candidates2[cand2])
        del candidates1[cand1]
        del candidates2[cand2]
    

    while len(candidates1)>0:
        print(len(list_of_edges), len(candidates1), len(candidates2))
        if len(candidates1) > 1:
            cand1 = random.randint(0,len(candidates1)-1)
        elif len(candidates1) == 1:
            cand1 = 0

        if graph.edges[candidates1[cand1]][0] in g1.nodes.keys() and graph.edges[candidates1[cand1]][1] in g1.nodes.keys():
            g1.connectTwoNodes(candidates1[cand1],graph.edges[candidates1[cand1]][0],graph.edges[candidates1[cand1]][1])
        elif graph.edges[candidates1[cand1]][0] in g1.nodes.keys():
            g1.addNode(graph.edges[candidates1[cand1]][1],graph.nodes[graph.edges[candidates1[cand1]][1]][0],graph.nodes[graph.edges[candidates1[cand1]][1]][1])
            for edge in graph.edgeLink[graph.edges[candidates1[cand1]][1]]:
                if (edge in list_of_edges) and (edge not in candidates1):
                    candidates1.append(edge)
            g1.connectTwoNodes(candidates1[cand1],graph.edges[candidates1[cand1]][0],graph.edges[candidates1[cand1]][1])
        elif graph.edges[candidates1[cand1]][1] in g1.nodes.keys():
            g1.addNode(graph.edges[candidates1[cand1]][0],graph.nodes[graph.edges[candidates1[cand1]][0]][0],graph.nodes[graph.edges[candidates1[cand1]][0]][1])
            for edge in graph.edgeLink[graph.edges[candidates1[cand1]][0]]:
                if (edge in list_of_edges) and (edge not in candidates1):
                    candidates1.append(edge)
            g1.connectTwoNodes(candidates1[cand1],graph.edges[candidates1[cand1]][0],graph.edges[candidates1[cand1]][1])
        else:
            print("Warning")

        if candidates1[cand1] in list_of_edges:
            list_of_edges.remove(candidates1[cand1])
        del candidates1[cand1]
        

    while len(candidates2)>0:
        print(len(list_of_edges), len(candidates1), len(candidates2))
        
        if len(candidates2) > 1:
            cand2 = random.randint(0,len(candidates2)-1)
        else:
            cand2 = 0
        
        if graph.edges[candidates2[cand2]][0] in g2.nodes.keys() and graph.edges[candidates2[cand2]][1] in g2.nodes.keys():
            g2.connectTwoNodes(candidates2[cand2],graph.edges[candidates2[cand2]][0],graph.edges[candidates2[cand2]][1])
        elif graph.edges[candidates2[cand2]][0] in g2.nodes.keys():
            g2.addNode(graph.edges[candidates2[cand2]][1],graph.nodes[graph.edges[candidates2[cand2]][1]][0],graph.nodes[graph.edges[candidates2[cand2]][1]][1])
            for edge in graph.edgeLink[graph.edges[candidates2[cand2]][1]]:
                if (edge in list_of_edges) and (edge not in candidates2):
                    candidates2.append(edge)
            g2.connectTwoNodes(candidates2[cand2],graph.edges[candidates2[cand2]][0],graph.edges[candidates2[cand2]][1])
        elif graph.edges[candidates2[cand2]][1] in g2.nodes.keys():
            g2.addNode(graph.edges[candidates2[cand2]][0],graph.nodes[graph.edges[candidates2[cand2]][0]][0],graph.nodes[graph.edges[candidates2[cand2]][0]][1])
            for edge in graph.edgeLink[graph.edges[candidates2[cand2]][0]]:
                if (edge in list_of_edges) and (edge not in candidates2):
                    candidates2.append(edge)
            g2.connectTwoNodes(candidates2[cand2],graph.edges[candidates2[cand2]][0],graph.edges[candidates2[cand2]][1])
        else:
            print("Warning")
        
        if candidates2[cand2] in list_of_edges:
            list_of_edges.remove(candidates2[cand2])
        del candidates2[cand2]
    
    g3 = Graph.Graph()
    while len(list_of_edges)>0:
        print(len(list_of_edges), len(candidates1), len(candidates2))
        if graph.edges[list_of_edges[0]][0] in g3.nodes.keys() and graph.edges[list_of_edges[0]][1] in g3.nodes.keys():
            g3.connectTwoNodes(list_of_edges[0],graph.edges[list_of_edges[0]][0],graph.edges[list_of_edges[0]][1])
        elif graph.edges[list_of_edges[0]][0] in g3.nodes.keys():
            g3.addNode(graph.edges[list_of_edges[0]][1],graph.nodes[graph.edges[list_of_edges[0]][1]][0],graph.nodes[graph.edges[list_of_edges[0]][1]][1])
            g3.connectTwoNodes(list_of_edges[0],graph.edges[list_of_edges[0]][0],graph.edges[list_of_edges[0]][1])
        elif graph.edges[list_of_edges[0]][1] in g3.nodes.keys():
            g3.addNode(graph.edges[list_of_edges[0]][0],graph.nodes[graph.edges[list_of_edges[0]][0]][0],graph.nodes[graph.edges[list_of_edges[0]][0]][1])
            g3.connectTwoNodes(list_of_edges[0],graph.edges[list_of_edges[0]][0],graph.edges[list_of_edges[0]][1])
        else:
            g3.addNode(graph.edges[list_of_edges[0]][1],graph.nodes[graph.edges[list_of_edges[0]][1]][0],graph.nodes[graph.edges[list_of_edges[0]][1]][1])
            g3.addNode(graph.edges[list_of_edges[0]][0],graph.nodes[graph.edges[list_of_edges[0]][0]][0],graph.nodes[graph.edges[list_of_edges[0]][0]][1])
            g3.connectTwoNodes(list_of_edges[0],graph.edges[list_of_edges[0]][0],graph.edges[list_of_edges[0]][1])

        list_of_edges.pop(0)

        
    
    g1.Dump2txt("chicago_split1")
    g2.Dump2txt("chicago_split2")
    g3.Dump2txt("chicago_split3")


graph = Graph.Graph("data/chicago/osm/chicago_osm")
print("Done reading")
split(graph)