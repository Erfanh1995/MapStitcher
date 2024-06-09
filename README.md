# Map Stitcher

Map Stitcher is an automated map conflation tool based on Graph Sampling. Inputs have to be in txt format, e.g. `chicago_osm_vertices.txt` and `chicago_osm_edges.txt` 

For maps check out https://www.mapconstruction.org/

```bash
python3 Merger.py <data_folder> <dataset_name> <map1> <map2>
```   
`<data_folder>` is the path to the data folder e.g. `data`

`<dataset_name>` is the name of the dataset folder inside `<data_folder>` e.g. `athens_small`

`<map1>` is the name of the map in `<data_folder>/<dataset_name>/map1` directory e.g. osm

`<map2>` is the name of the map in `<data_folder>/<dataset_name>/map2` directory

