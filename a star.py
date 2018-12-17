import operator
import random


class Board(object):
    """
    Esta classe representa uma configuração do tabuleiro do
    quebra-cabeça. O tabuleiro é um estado no problema de busca.
    
    O tabuleiro tem 9 posições (em inglês tiles), sendo 8 posições dedicadas
    aos números de 1 até 8 e uma posição especial "x" que representa a posição
    vazia.
 
    O tabuleiro é representado de forma linear, por exemplo,
    [1, 2, 3, 4, 5, 6, 7, 8, "x"], que visualmente representa o tabuleiro:

                                   1  2  3
                                   4  5  6
                                   7  8  x
    """

    def __init__(self, tiles):
        
        self.goal = [1,2,3,4,5,6,7,8,"x"]
        self.tiles = tiles

    def is_goal(self):
        #print(self.tiles)
        if self.tiles==self.goal:
            return True
        return False
    
    def heuristic(self):
        count=0
        lista=map(operator.eq, self.tiles, [1, 2, 3, 4, 5, 6, 7, 8, "x"])
        for i in lista:
            if(i==False):
                count+=1
        return count
        

    def get_neighbors(self):
        index = 0
        neighbors=[]
        for i in range(len(self.tiles)):
            if self.tiles[i] == 'x':
                index = i
        if (index==0 or index==1 or index==3 or index==4 or index==6 or index==7):
            atual = self.tiles.copy()
            atual[index] = atual[index+1]
            atual[index+1] = 'x'
            neighbors.append(Board(atual))
        if(index==1 or index==2 or index==4 or index==5 or index==7 or index==8):
            atual = self.tiles.copy()
            atual[index] = atual[index-1]
            atual[index-1] = 'x'
            neighbors.append(Board(atual))
        if(index==0 or index==1 or index==2 or index==3 or index==4 or index==5):
            atual = self.tiles.copy()
            atual[index] = atual[index+3]
            atual[index+3] = 'x'
            neighbors.append(Board(atual))
        
        if(index==3 or index==4 or index==5 or index==6 or index==7 or index==8):
            atual = self.tiles.copy()
            atual[index] = atual[index-3]
            atual[index-3] = 'x'
            neighbors.append(Board(atual))
        
        return neighbors
            
    # Os métodos a seguir dessa classe não devem ser modificados
    def __eq__(self, other):
        return self.tiles == other.tiles

    def __hash__(self):
        return hash(tuple(self.tiles))

    def __str__(self):
        return str(self.tiles)

    def __repr__(self):
        return str(self.tiles)

    def print_board(self):
        print(self.tiles[:3])
        print(self.tiles[3:6])
        print(self.tiles[6:])


class Node(object):
    """
    Esta classe representa um nó na busca. Cada nó contém um estado (tabuleiro),
    um custo e o pai do nó, este último é a referência para um outro nó.

    Esta classe não deve ser modificada.
    """
    
    def __init__(self, state, cost):
        """
        Construtor.

        state é um objeto da classe Board.
        cost é um número que representa o custo do nó.
        """
        self.state = state
        self.cost = cost
        self.parent = None

    def __str__(self):
        return str(self.state.tiles) + " - " + str(self.cost)

    def __repr__(self):
        return str(self.state.tiles) + " - " + str(self.cost)


class AStar(object):
    """
    Esta classe é responsável por fazer a busca A*. Ela recebe um estado
    inicial no construtor indicando a configuração inicial do tabuleiro do
    quebra cabeça.
    """
    
    def __init__(self, initial_state):
        """
        Construtor.

        initial_state é o estado inicial, ou seja, a configuração inicial do
        tabuleiro.

        No construtor também é iniciada a fronteira e o conjunto dos
        explorados. Note que a fronteira é uma lista de objetos da classe Node.
        """
        self.initial_state = initial_state
        self.frontier = [Node(self.initial_state, 0 + self.initial_state.heuristic())]
        self.explored = set()
        self.current_node = None

    def choose_from_frontier(self):
        menor = self.frontier[0]
        for i in self.frontier:
            if i.cost < menor.cost:
                menor = i
        self.frontier.remove(menor)
        return menor
    
    def update_frontier(self):
        vizinhos = self.current_node.state.get_neighbors()
        for vizinho in vizinhos:
            node = Node(vizinho, vizinho.heuristic())
            if not(self.is_neighbor_in_frontier(node.state)):
                if not(node.state in self.explored):
                    node.parent = self.current_node
                    self.frontier.append(node)
            
            

    def is_neighbor_in_frontier(self, neighbor):
        """
        Este método avalia se algum nó da fronteira contém o estado (neighbor).
        Você pode utilizar este método para implementar o update_frontier.
        """
        for node in self.frontier:
            if node.state == neighbor:
                return True
        return False

    def get_path(self, node):
        lista = [node.state]
        while self.initial_state != node.state:
            lista.append(node.parent.state)
            node = node.parent

        return lista.__reversed__()

    def search(self):
        """
        Este método executa a busca A* para resolver o problema do
        quebra-cabeça de 8 números.

        Atenção: Para algumas configurações de tabuleiro, a solução pode ser
        impossível, causando um loop infinito.
        """
        while True:
            if len(self.frontier) == 0:
                return False

            self.current_node = self.choose_from_frontier()

            self.explored.add(self.current_node.state)
            if self.current_node.state.is_goal():
                return self.current_node

            self.update_frontier()
            

if __name__ == "__main__":
    tiles = [3, 2, 8, 1, 5, 4, 7, 6, "x"]
    initial_state = Board(tiles)

    astar = AStar(initial_state)
    final_node = astar.search()
    path = astar.get_path(final_node)

    for state in path:
        state.print_board()
        print("---")
