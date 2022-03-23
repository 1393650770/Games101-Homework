#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        Vector2D step=(end-start)/float(num_nodes-1);
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        for (size_t i = 0; i < num_nodes; i++)
        {
            Mass* mass =new Mass(start+i*step,node_mass,false);
            mass->velocity=Vector2D(0,0);
            if (i>0)
            {
                Spring* spring =new Spring(masses.back(),mass,k);
                springs.push_back(spring);
            }
            masses.push_back(mass);
        }
         
//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D ba=s->m2->position-s->m1->position;
            Vector2D force = -s->k*(ba)*(ba.norm()-s->rest_length)/ba.norm();
            s->m1->forces -= force;
            s->m2->forces += force;
        }

        bool IsSemi_implict=false;

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces+=gravity*m->mass;
                // TODO (Part 2): Add global damping
                if(IsSemi_implict) m->velocity+=m->forces/m->mass*delta_t;
                m->position += m->velocity*delta_t;
                if(!IsSemi_implict) m->velocity+=m->forces/m->mass*delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D ba=s->m2->position-s->m1->position;
            Vector2D force = -s->k*(ba)*(ba.norm()-s->rest_length)/ba.norm();
            s->m1->forces -= force;
            s->m2->forces += force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                Vector2D lastPos= m->position;
                m->forces+=gravity*m->mass;
                m->position += (1-0.0005f)*(m->position-m->last_position)+m->forces/m->mass*delta_t*delta_t;
                m->last_position=lastPos;
                
            }
        }
    }
}
