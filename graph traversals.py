#!/usr/bin/env python
# coding: utf-8

# In[1]:


get_ipython().run_line_magic('pylab', 'inline')


# In[2]:


n, m = 20, 20  # number of rows and columns respectively.


# Create a matrix to represent the cells of the grid
grid_cells = np.zeros((20,20))

# Random obstacles for this example. 

grid = [[0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.],
        [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,1.,1.,0.,0.],
        [0.,0.,0.,0.,1.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.,1.,1.,0.,0.],
        [0.,0.,0.,1.,1.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.],
        [0.,0.,0.,1.,1.,1.,0.,0.,0.,0.,0.,0.,1.,1.,0.,0.,0.,0.,0.,0.],
        [0.,0.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,1.,1.,0.,0.,0.,0.,0.,0.],
        [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.],
        [0.,0.,0.,0.,0.,0.,0.,1.,1.,1.,1.,0.,1.,1.,1.,0.,0.,0.,0.,0.],
        [0.,0.,0.,1.,1.,0.,0.,1.,1.,1.,1.,1.,1.,1.,1.,0.,0.,0.,0.,0.],
        [0.,0.,0.,1.,1.,0.,0.,1.,1.,1.,1.,1.,1.,0.,0.,0.,0.,0.,0.,0.],
        [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,1.,1.,0.,0.,0.,0.,0.],
        [0.,0.,0.,0.,0.,0.,0.,1.,1.,0.,0.,0.,0.,1.,1.,0.,0.,0.,0.,0.],
        [0.,0.,0.,0.,1.,1.,0.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.],
        [0.,0.,0.,0.,1.,1.,0.,0.,0.,0.,0.,1.,1.,0.,0.,1.,1.,1.,1.,0.],
        [0.,0.,0.,0.,1.,1.,0.,0.,0.,0.,0.,1.,1.,0.,0.,1.,1.,1.,1.,0.],
        [0.,0.,1.,1.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,0.,1.,1.,1.,1.,0.],
        [0.,0.,1.,1.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.],
        [0.,0.,1.,1.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.],
        [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.],
        [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]]
        


# In[3]:


# Plot the grid
from matplotlib.colors import ListedColormap
def draw_grid():
    fig, ax = plt.subplots()
    cmap = ListedColormap(['0.9', 'black']) # Colors (0.9 is the almost white in gray scale)
    ax.imshow(grid, cmap=cmap, origin='lower')
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
    ax.set_xticks(np.arange(-0.5, m, 1));
    ax.set_yticks(np.arange(-0.5, n, 1));

draw_grid()


# In[22]:


import networkx as nx
# This function automatically creates the graph in a grid.
G = nx.grid_2d_graph(n, m) 
print("Number of nodes in a full grid (n*m): ",len(G.nodes()))
# Delete nodes with obstacles
for i in range(n):
    for j in range(m):
        # if the node is an obstacle
        if grid[i][j] == 1:  
            G.remove_node((i,j))

print("Number of nodes after removing obstacles: ",len(G.nodes()))
print("Nodes:", G.nodes())


# In[23]:


# Position of the nodes
pos = {node:(node[1], node[0]) for node in G.nodes()}  # by making (x,y) = (j, i), where i and j iterate over the columns and the rows respectively.
nx.draw(G, pos, font_size=7, with_labels=True, node_size=100, node_color="g")


# In[24]:


# Plot grid
draw_grid()

# Plot the graph
pos = {node:(node[1], node[0]) for node in G.nodes()}  # by making (x,y) = j, -i, where i and j iterate over the columns and the rows respectively.
nx.draw(G, pos, font_size=5, with_labels=True, node_size=200, node_color="g")


# In[25]:


start_node =  (18, 17)
end_node = (2,2)

# Run BFS to generate a tree
bfs_tree = nx.bfs_tree(G, source=start_node)


# In[26]:


# plot the result of BSF
draw_grid()
nx.draw(bfs_tree, pos = pos)


# In[27]:


# Pick the last element and iterate through its predecessors
path = [end_node]   # A path in a graph is a set of connected nodes
current = end_node

# iterate through its predecessors until finding source node
while current != start_node:
    # Predecesors of the current node        
    for pre in bfs_tree.predecessors(current):
        current = pre
    # add the predecessor to the path
    path.append(pre)
        
# The current path starts in the goal node and ends at the start node. So we invert it
path = path[::-1]

# Correct path
print(path)


# In[28]:


# Edges of the path
E = [(path[i], path[i+1])for i in range(len(path)-1)]

draw_grid()
nx.draw_networkx_nodes(path, pos)
nx.draw_networkx_edges(G, edgelist=E, pos=pos,edge_color='r')


# In[ ]:





# In[29]:


source =  (18, 17)
target = (2,2)
def dist(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

path=nx.astar_path(G, (17,18), (1,2), dist)
print(path)


# In[30]:


E = [(path[i], path[i+1])for i in range(len(path)-1)]

draw_grid()
nx.draw_networkx_nodes(path, pos)
nx.draw_networkx_edges(G, edgelist=E, pos=pos,edge_color='r')


# In[ ]:




