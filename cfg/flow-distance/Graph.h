#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <queue>
#include <set>

namespace graph {
  template<class T, class V>
  class Graph;
  template<class T, class V>
  class Node;
  
  template<class T, class V>
  class Edge 
  {
  private:
    Node<T, V> src;
    Node<T, V> dest;
    V data;
    
  public:
    typedef typename Node<T, V> node;  
    typedef typename Edge<T, V> edge;
    
    Node(const node& src, const node& dest, const V& data = V()) : src(src), dest(dest), data(data){}
    Node(const edge& other) : src(other.src), dest(other.dest), data(other.data){}
    
    node get_src() const { return src; }
    node get_dest() const { return dest; }
    const V& get_data() const { return data; }
  };
  
  template<class T, class V>
  class Node
  {
    friend class Graph<T, V, true>;
    friend class Graph<T, V, false>;
  private:
    int index;
    std::vector<Edge<T, V> > edges;
    std::vector<Node<T, V> > neighbors;
    T data;
  public:
    Node(int index = -1, const T& data = T()) : index(index), data(data){}
    Node(const Node& other) : index(other.index), data(other.data){}
    
    typedef typename Edge<T, V> edge;
    typedef typename std::vector<edge::iterator iterator;
    typedef typename std::vector<edge>::const_iterator const_iterator;
    typedef typename std::vector<edge>::reverse_iterator reverse_iterator;
    typedef typename std::vector<edge>::const_reverse_iterator const_reverse_iterator;
    typedef typename std::vector<edge>::size_type size_type;

    void addEdge(const edge& e) {
      edges.push_back(e);
    }

    iterator begin() { return edges.begin(); }
    const_iterator begin() const { return edges.begin(); }

    iterator end() { return edges.end(); }
    const_iterator end() const { return edges.end(); }

    reverse_iterator rbegin() { return edges.rbegin(); }
    const_reverse_iterator rbegin() const { return edges.begin(); }

    reverse_iterator rend() { return edges.rend(); } 
    const_reverse_iterator rend() const { return edges.rend(); }

    size_type degree() const { return edges.size(); }

    int get_index() const { return index; }
    const T& get_data() const { return data; }
    const Node<T, V>& get_neighbors() const { 
      return neighbors; 
    }
    //void set_data(const T& data) { this->data = data; }
  };

  template<class T, class V>
  class Graph
  {
  private:
    int index = 0;
    
    std::vector<Node<T, V> > nodes;
    typedef Node<T, V> node;
    typedef Edge<T, V> edge;
  public:
    typedef typename std::vector<node>::iterator iterator;
    typedef typename std::vector<node>::const_iterator const_iterator;
    typedef typename std::vector<node>::reverse_iterator reverse_iterator;
    typedef typename std::vector<node>::const_reverse_iterator const_reverse_iterator;
    typedef typename std::vector<node>::size_type size_type;

    Graph(int v) {
      nodes.reserve(v);
    }

    iterator begin() { return nodes.begin(); }
    const_iterator begin() const { return nodes.begin(); }

    iterator end() { return nodes.end(); }
    const_iterator end() const { return nodes.end(); }

    reverse_iterator rbegin() { return nodes.rbegin(); }
    const_reverse_iterator rbegin() const { return nodes.rbegin(); }

    reverse_iterator rend() { return nodes.rend(); }
    const_reverse_iterator rend() const { return nodes.rend(); }

    int addNode(const T& node_data) {
      int n = index;
      nodes.push_back(Node(n, node_data);
      index++;  
      return n;
    }

    void addEdge(int src, int dest, const V& edge_data = V()) {
      edge e(src, dest, edge_data);
      nodes[src].addEdge(e);
      nodes[dest].addEdge(e);
    }

    void computeMinDistance(int src, std::map<T, unsigned>& distances) {
      std::queue<node> q;
      std::set<node> visited;
      const src_node = nodes[src];
      
      q.push(src_node);
      distances[src_node.get_data()] = 0;
      visited.insert(src_node);
    
      while (!q.empty()) {
        node bb = q.front();
        q.pop();
        unsigned distance = distances[bb];

        std::vector<node> &nodes = bb.get_neighbors();
        for (std::vector<node>::iterator it = nodes.begin(), ite = nodes.end();
           it != ite; ++it) {
          std::vector<node>::iterator f = visited.find(*it);
          if (f == visited.end()) {
            visited.insert(*it);
            distances[it->get_data()] = distance + 1;
            q.push(*it);
          } else if (distances[f->get_data()] > distance + 1) {
            distances[it->get_data()] = distance + 1;
          }
        }
      }
    }
    
    std::vector<node>& get_nodes() const { return nodes.size(); }

    node& operator[](int i) { return nodes[i]; }
    const node& operator[](int i) const { return nodes[i]; }
  };
}
#endif 
