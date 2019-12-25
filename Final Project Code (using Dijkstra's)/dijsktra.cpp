/*
 * dijsktra.cpp
 *
 *  Created on: Dec 4, 2019
 *      Author: kgandhi
 */

#include "dijkstra.h"

dijkstra::dijkstra(){

}

vector<int> dijkstra::findPath(int startNode, int endNode){
//	int endNode = 35;
	//	cout << "Enter the destination - " << endl;
	//	cin >> endNode;

//	int startNode = 0;

	int Q[36][3];

	for(int i = 0; i < 36; i++){   // setting up queue
		Q[i][0] = -1;              // setting penultimate vertex = null / -1 for int
		if(i == startNode){
			Q[i][1] = 0;           //setting distance for source
		}
		else{
			Q[i][1] = INT_MAX;     // setting rest nodes to infinty
		}
		Q[i][2] = 0;               //setting everything not visited
	}

	for(int i = 0; i< 36; i++){


		int min = INT_MAX;    //min distance not visited in queue
		int minIndex=0;         // index of minimum distance
		for(int j =0; j < 36; j++){        // go through all the nodes which have Q[j][2] = 0
			if(Q[j][2] == 0){			   // checks if the node is visited or not
				if(Q[j][1] < min){
					min = Q[j][1];        //resets a new min (lesser than prev min)
					minIndex = j;		  // resets the index of min
				}
			}
		}
		Q[minIndex][2] = 1;               //sets the min node to visited


		//get adjacent nodes from matrix

		vector<int> adjacent;      // array of adjacent nodes

		for(int j = 0; j<36; j++){
			if(fieldArr[minIndex][j] > 0){
				adjacent.push_back(j);
			}
		}

		for(int j = 0; j<36; j++){
			if(Q[j][2] == 0 && contains(adjacent, j)){       //check if node is not visited and adjacent
				if(Q[minIndex][1] + fieldArr[minIndex][j] <= Q[j][1]){
					Q[j][1] = Q[minIndex][1] + fieldArr[minIndex][j];
					Q[j][0] = minIndex;
				}
			}
		}

		//int j = 0;


	}

	vector<int> path;
	path.push_back(endNode);

	while(endNode != startNode){ // endNode will be mutated to penUltimate
		path.push_back(Q[endNode][0]);
		endNode = Q[endNode][0];
	}

	int size=path.size();
	for(int i=size-1;i>=0;i--){
		if(i==0){
			//cout<< path[i];
			Serial.print("(");
			Serial.print(nodeCoordinateX(path[i]));
			Serial.print(",");
			Serial.print(nodeCoordinateY(path[i]));
			Serial.print(")");
		}
		else{
			//cout<< path[i] << "->";
			Serial.print("(");
			Serial.print(nodeCoordinateX(path[i]));
			Serial.print(",");
			Serial.print(nodeCoordinateY(path[i]));
			Serial.print(")->");
		}
	}
	cout << endl;
	return path;
}

bool dijkstra::contains(vector<int> adjNodes, int node){
	bool found = false;
	int size = adjNodes.size();
	for(int i = 0; i< size; i++){
		if(node == adjNodes[i]){
			found = true;
		}

	}
	return found;
}

int dijkstra::nodeCoordinateX(int node){
	int xCoord = 0;
	if(node%6 == 1){
		xCoord = 1;
	}
	else if(node%6 == 2){
		xCoord = 2;
	}
	else if(node%6 == 3){
		xCoord = 3;
	}
	else if(node%6 == 4){
		xCoord = 4;
	}
	else if(node%6 == 5){
		xCoord = 5;
	}

	return xCoord;
}

int dijkstra::nodeCoordinateY(int node){
	int yCoord = 0;
	if(node == 11 || node == 10 || node == 9 || node == 8 || node == 7 || node == 6){
		yCoord = 1;
	}
	else if(node == 12 || node == 13 || node == 14 || node == 15 || node == 16 || node == 17){
		yCoord = 2;
	}
	else if(node == 23 || node == 22 || node == 21 || node == 20 || node == 19 || node == 18){
		yCoord = 3;
	}
	else if(node == 24 || node == 25 || node == 26 || node == 27 || node == 28 || node == 29){
		yCoord = 4;
	}
	else if(node == 35 || node == 34 || node == 33 || node == 32 || node == 31 || node == 30){
		yCoord = 5;
	}

	return yCoord;
}

void dijkstra::blockNode (int node){
	for(int i = 0; i<36; i++){
		fieldArr[i][node] = 0;
	}
}

