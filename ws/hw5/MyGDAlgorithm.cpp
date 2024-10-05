#include "MyGDAlgorithm.h"


double euclidian(const Eigen::Vector2d& q, const Eigen::Vector2d& p) {
			return sqrt(pow((q[0]-p[0]), 2) + pow(q[1]-p[1],2));
			// return 1;
		}

Eigen::Vector2d centroid(const amp::Obstacle2D obs)
{

	std::vector<Eigen::Vector2d> vertices = obs.verticesCCW();
	int vertexCount = vertices.size();
    Eigen::Vector2d centroid; centroid << 0, 0; 
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices except last
    int i=0;
    for (i=0; i<vertexCount-1; ++i)
    {
        x0 = vertices[i][0];
        y0 = vertices[i][1];
        x1 = vertices[i+1][0];
        y1 = vertices[i+1][1];
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid[0] += (x0 + x1)*a;
        centroid[1] += (y0 + y1)*a;
    }
    // Do last vertex separately to avoid performing an expensive
    // modulus operation in each iteration.
    x0 = vertices[i][0];
    y0 = vertices[i][1];
    x1 = vertices[0][0];
    y1 = vertices[0][1];
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid[0] += (x0 + x1)*a;
    centroid[1] += (y0 + y1)*a;

    signedArea *= 0.5;
    centroid[0] /= (6.0*signedArea);
    centroid[1] /= (6.0*signedArea);

    return centroid;
}

double dist_to_obs(const Eigen::Vector2d& q, const amp::Obstacle2D obs) {
			//TODO: might have to edit this so that it takes into accounts all points on the obstacle instead of just the vertices, but since it's convex I think it's mostly fine
			std::vector<Eigen::Vector2d> verts = obs.verticesCCW();
			double dist = euclidian(q, verts[0]);
			// for (Eigen::Vector2d vert : verts){
			// 	if (euclidian(q, vert) < dist){ dist=euclidian(q, vert); } 
			// }
			for (int i = 0; i < verts.size(); i++){
				Eigen::Vector2d current = verts[i];
				Eigen::Vector2d next;
				if (i == verts.size()-1){
					next = verts[0];
				}
				else{
					next = verts[i + 1];
				}
				for (double t = current[0]; t <= next[0]; t+=.1 ){
					Eigen::Vector2d point; point << current[0] + t, current[1] + t*((next[1]-current[1])/(next[0]-current[0]));
					if (euclidian(point, q) < dist){dist=euclidian(q, point);}
				}
			
		}
		return dist;
}

Eigen::Vector2d closest_vert(const Eigen::Vector2d& q, const amp::Obstacle2D obs) {
			//TODO: might have to edit this so that it takes into accounts all points on the obstacle instead of just the vertices, but since it's convex I think it's mostly fine
			std::vector<Eigen::Vector2d> verts = obs.verticesCCW();
			//double dist = euclidian(q, verts[0]);
			Eigen::Vector2d poop = verts[0];
			//Eigen::Vector2D closest = verts[0];
			// for (Eigen::Vector2d vert : verts){
			// 	if (euclidian(q, vert) < euclidian(q, poop)){ poop=vert; } 
			// }
			for (int i = 0; i < verts.size(); i++){
				Eigen::Vector2d current = verts[i];
				Eigen::Vector2d next;
				if (i == verts.size()-1){
					next = verts[0];
				}
				else{
					next = verts[i + 1];
				}
				for (double t = current[0]; t <= next[0]; t+=.1 ){
					Eigen::Vector2d point; point << current[0] + t, current[1] + t*((next[1]-current[1])/(next[0]-current[0]));
					if (euclidian(point, q) < euclidian(poop, q)){poop = point;}
				}
			//std::cout<< "current q: " << q[0] << ", " << q[1] << "\n";
			//std::cout<< "closest obstacle: " << poop[0] << ", " << poop[1] << "\n";
		
		}
		return poop;
}


Eigen::Vector2d projected_closest(const Eigen::Vector2d& q, const amp::Obstacle2D obs) {
			//TODO: might have to edit this so that it takes into accounts all points on the obstacle instead of just the vertices, but since it's convex I think it's mostly fine
			std::vector<Eigen::Vector2d> verts = obs.verticesCCW();
			//double dist = euclidian(q, verts[0]);
			Eigen::Vector2d closest =  closest_vert(q, obs);
			Eigen::Vector2d cent = centroid(obs);
			//double dist = euclid(closest, cent);
			double x_dist = cent[0] - closest[0];
			double y_dist = cent[1] - closest[1];
			Eigen::Vector2d point; point << closest[0] + x_dist*3/4, closest[0] + y_dist*3/4;
			//find the point that is exactly halfway between the centroid and closest vert
		return point;
}

// Implement your plan method here, similar to HW2:


amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    Eigen::Vector2d q = problem.q_init;
    double epsilon = .25;
	int count = 0;
	//std::cout<< "Number of obstacles: " << problem.obstacles.size() << "\n";
    while (euclidian(q, problem.q_goal) > epsilon){
        //ATTRACTIVE FORCE
			Eigen::Vector2d att;
			double euclid = euclidian(q, problem.q_goal);
			if (euclid <= d_star){
				att[0] =  zetta * (q[0] - problem.q_goal[0]);
                att[1] =  zetta * (q[1] - problem.q_goal[1]);
			}
			else{
				att[0] = d_star * zetta * (q[0] - problem.q_goal[0]) / euclid ;
                att[1] = d_star * zetta * (q[1] - problem.q_goal[1]) / euclid ;
			}
            
			
			count++;
			if (count == 10000){
				break;
			}
            
			//REPULSIVE FORCE
			Eigen::Vector2d rep; rep << 0, 0;
			std::vector<amp::Obstacle2D> obstacles = problem.obstacles;
			std::vector<Eigen::Vector2d> coeffs;
			coeffs.push_back(Eigen::Vector2d({1, 0}));
			coeffs.push_back(Eigen::Vector2d({1, 0}));
			coeffs.push_back(Eigen::Vector2d({1, 20}));
			coeffs.push_back(Eigen::Vector2d({1, 20}));
			int wsid = 0; //for hw5ws1 and hw2ws1
			//std::cout<<"SIZE OF OBS" << problem.obstacles.size() << "\n";
			if (problem.obstacles.size() >= 8){
				wsid = 1; //forhw2ws2
				Q_star = 5;
				eta = 0.8;
			}
			//wsid = 1;
			//Q_star = std::min(euclid, Q_star);
			for (amp::Obstacle2D obs : obstacles){
				//std::cout<<"checking obs" << "\n";
				double di = dist_to_obs(q, obs);
				//Eigen::Vector2d closest = closest_vert(q, obs);
				Eigen::Vector2d closest = projected_closest(q, obs);

				if (di <= Q_star){
					//std::cout<<"adding to rep" << "\n";
					// rep[0] += eta * coeffs[0][wsid] * (1/Q_star - 1/di)*((q[0]-closest[0])/di)/pow(di, 2);
                    // rep[1] += eta * coeffs[1][wsid] * (1/Q_star - 1/di)*((q[1]-closest[1])/di)/pow(di, 2);
					rep[0] += eta * coeffs[0][wsid] * (1/Q_star - 1/di)*((q[0]-closest[0])/di)/pow(di, 2);
                    rep[1] += eta * coeffs[1][wsid] * (1/Q_star - 1/di)*((q[1]-closest[1])/di)/pow(di, 2);
				}
				Eigen::Vector2d cent = centroid(obs);
				di = euclidian(cent, q);
				if (di <= Q_star){
					// rep[0] +=  eta * coeffs[2][wsid] * (1/Q_star - 1/di)*((q[0]-cent[0])/di)/pow(di, 2);
                    // rep[1] +=  eta * coeffs[3][wsid] * (1/Q_star - 1/di)*((q[1]-cent[1])/di)/pow(di, 2);
					rep[0] +=  eta * coeffs[2][wsid] *  (1/Q_star - 1/di)*((q[0]-cent[0])/di)/pow(di, 2);
                    rep[1] +=  eta * coeffs[3][wsid]*(1/Q_star - 1/di)*((q[1]-cent[1])/di)/pow(di, 2);
				}

			}
			//std::cout<< "repulsive force: " << rep[0] << ", " << rep[1] << "\n";
			double alpha = 0.3;
			Eigen::Vector2d changePos; changePos << att[0] + rep[0], att[1] + rep[1];
			double lenChangePos = sqrt(pow(changePos[0], 2) + pow(changePos[1], 2));
			//TODO: INSERT CASE FOR IF IT GETS STUCK AT LOCAL MIN
			//but before that, i would first fix the fact that it literally goes through the obstacles in WS3
			if (abs(changePos[0]) < .05 && abs(changePos[1])< .05 ){
				Eigen::Vector2d changePos; changePos << att[0], att[1];
				double lenChangePos = sqrt(pow(changePos[0], 2) + pow(changePos[1], 2));
				q[0] = q[0] - alpha/lenChangePos*(att[0]);
            	q[1] = q[1] - alpha/lenChangePos*(att[1]);
				path.waypoints.push_back(q);
				continue;
			}
			q[0] = q[0] - .55/lenChangePos*(att[0] + rep[0]);
            q[1] = q[1] - .55/lenChangePos*(att[1] + rep[1]);
			//std::cout<< "pushing back" << q[0] << ", " << q[1] << "\n";
			path.waypoints.push_back(q);
        //calculate vector representing gradient
        //take a step in that direction
        //push back new q
        //update q
    }
    

    path.waypoints.push_back(problem.q_goal);
    return path;
}


