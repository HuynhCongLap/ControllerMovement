#include "Application.h"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
using namespace std;

void DestructionListener::SayGoodbye(b2Joint* joint) {
	if (app->m_mouseJoint == joint) app->m_mouseJoint = NULL;
	else app->JointDestroyed(joint);
}

Application::Application() {

    // Gravity and world
	b2Vec2 gravity;
	gravity.Set(0.0f, -9.81f);
	m_world = new b2World(gravity);

	// Init
	m_textLine = 30;
	m_mouseJoint = NULL;
	m_COM = NULL;
	m_ball = NULL;
	m_pointCount = 0;
	m_destructionListener.app = this;
	m_world->SetDestructionListener(&m_destructionListener);
	m_world->SetContactListener(this);
	m_world->SetDebugDraw(&g_debugDraw);
	m_world->SetAllowSleeping(true);
	m_world->SetWarmStarting(true);
	m_world->SetContinuousPhysics(true);
	m_world->SetSubStepping(false);
	m_stepCount = 0;
	memset(&m_maxProfile, 0, sizeof(b2Profile));
	memset(&m_totalProfile, 0, sizeof(b2Profile));
	m_lastShoot = glfwGetTime();
	m_lastUpdate = glfwGetTime();
	m_walkDistance = 0.0f;

	// Ground body
	{
        b2BodyDef bodyDef;
        m_groundBody = m_world->CreateBody(&bodyDef);
		b2EdgeShape shape;
		shape.Set(b2Vec2(-5.0f, 0.0f), b2Vec2(1000.0f, 0.0f)); // 1km long ground
		m_groundBody->CreateFixture(&shape, 0.0f);
		m_groundBody->GetFixtureList()[0].SetUserData((void*)ID_GROUND);
	}

	// Biped
	m_biped = new Biped(m_world);

}

Application::~Application() {
	// By deleting the world, we delete the ball, ground, creature bodies and joints, etc.
	delete m_world;	m_world = NULL;
	delete m_biped; m_biped = NULL;
}

void Application::PreSolve(b2Contact* contact, const b2Manifold* oldManifold) {
	const b2Manifold* manifold = contact->GetManifold();
	if (manifold->pointCount == 0) return;

	b2Fixture* fixtureA = contact->GetFixtureA();
	b2Fixture* fixtureB = contact->GetFixtureB();

	b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
	b2GetPointStates(state1, state2, oldManifold, manifold);

	b2WorldManifold worldManifold;
	contact->GetWorldManifold(&worldManifold);

	for (int32 i = 0; i < manifold->pointCount && m_pointCount < k_maxContactPoints; ++i) {
		ContactPoint* cp = m_points + m_pointCount;
		cp->fixtureA = fixtureA;
		cp->fixtureB = fixtureB;
		cp->position = worldManifold.points[i];
		cp->normal = worldManifold.normal;
		cp->state = state2[i];
		cp->normalImpulse = manifold->points[i].normalImpulse;
		cp->tangentImpulse = manifold->points[i].tangentImpulse;
		cp->separation = worldManifold.separations[i];
		++m_pointCount;
	}
}

void Application::DrawTitle(const char *string) {
    g_debugDraw.DrawString(5, 5, string);
    m_textLine = 3 * DRAW_STRING_NEW_LINE;
}

class QueryCallback : public b2QueryCallback {
public:
	QueryCallback(const b2Vec2& point) 	{
		m_point = point;
		m_fixture = NULL;
	}
	bool ReportFixture(b2Fixture* fixture) override {
		b2Body* body = fixture->GetBody();
		if (body->GetType() == b2_dynamicBody) {
			bool inside = fixture->TestPoint(m_point);
			if (inside) {
				m_fixture = fixture;
				// We are done, terminate the query.
				return false;
			}
		}
		// Continue the query.
		return true;
	}
	b2Vec2 m_point;
	b2Fixture* m_fixture;
};

void Application::MouseDown(const b2Vec2& p) {
	m_mouseWorld = p;
	if (m_mouseJoint != NULL) return;

	// Make a small box.
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.lowerBound = p - d;
	aabb.upperBound = p + d;

	// Query the world for overlapping shapes.
	QueryCallback callback(p);
	m_world->QueryAABB(&callback, aabb);

	if (callback.m_fixture) {
		b2Body* body = callback.m_fixture->GetBody();
		b2MouseJointDef md;
		md.bodyA = m_groundBody;
		md.bodyB = body;
		md.target = p;
		md.maxForce = 1000.0f * body->GetMass();
		m_mouseJoint = (b2MouseJoint*)m_world->CreateJoint(&md);
		body->SetAwake(true);
	}
}

void Application::MouseUp(const b2Vec2& p) {
	if (m_mouseJoint) {
		m_world->DestroyJoint(m_mouseJoint);
		m_mouseJoint = NULL;
	}
}

void Application::MouseMove(const b2Vec2& p) {
	m_mouseWorld = p;
	if (m_mouseJoint) m_mouseJoint->SetTarget(p);
}

void Application::LaunchBall() {
	if (m_ball)	{
		m_world->DestroyBody(m_ball);
		m_ball = NULL;
	}

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	double angleAutour = ((rand() % 180) / 180.0f) * b2_pi;
	bd.position = m_biped->getCOM() + b2Vec2(cos(angleAutour)*2.0f,sin(angleAutour)*2.0f+0.5f);
	bd.bullet = true;
	m_ball = m_world->CreateBody(&bd);
	b2Vec2 cible = m_biped->getCOM();
	cible.y += (rand() % 10) / 10.0f;
	b2Vec2 velocity ((cible.x-bd.position.x)*5.0f,(cible.y-bd.position.y)*5.0f);
	m_ball->SetLinearVelocity(velocity);

	b2CircleShape circle;
	circle.m_radius = 0.06f;

	b2FixtureDef fd;
	fd.shape = &circle;
	fd.density = 8.0f;
	fd.restitution = 0.0f;
	fd.userData = (void*)ID_BALL;

	m_ball->CreateFixture(&fd);
}

void Application::setOptimizationData(const float * data) {

    /// ============ TODO PARTIE III ============= ///
    /*
    Dans cette procédure vous devez affecter à vos paramètres à optimiser le contenu du tableau data
    Ex. m_biped->getStateMachine()->m_states[0].targetAngles[0] = data[0]; pour affecter la première valeur du tableau au
    premier angle du premier état de la machine à états
    */


     for(int i=0; i<=6 ;i++)
        m_biped->getPDControllers()[i]->setKdGain(data[i]);


     for(int i=7; i<=13 ;i++)
        m_biped->getPDControllers()[i-7]->setKpGain(data[i]);
    /* Vous avez par exemple accès à :
    nombre d'états : m_biped->getStateMachine()->m_nbStates
    nombre d'articulations (si au moins 1 état) : m_biped->getStateMachine()->m_states[0].targetAngles.size()
    angle i d'un état j : m_biped->getStateMachine()->m_states[i].targetAngles[j]
    le temps de transition de l'état i : m_biped->getStateMachine()->m_states[i].transitionTime
    les PD controlleurs : m_biped->getPDControllers()
    les gains du controlleur i : PDcontrollers[i]->setGains(...) , PDcontrollers[i]->getKpGain() et PDcontrollers[i]->getKdGain()
    */

    /// ========================================== ///

}

void Application::getOptimizationData(float * data) {

    /// ============ TODO PARTIE III ============= ///
    /*
    Dans cette procédure vous devez affecter dans votre tableau les valeurs des paramètres à optimiser
    Ex. data[0] = m_biped->getStateMachine()->m_states[0].targetAngles[0]; pour affecter la valeur du premier angle

    du premier état de la machine à états à la première valeur du tableau
    */
     for(int i=0; i<=6 ;i++)
        data[i] =  m_biped->getPDControllers()[i]->getKdGain();


     for(int i=7; i<=13 ;i++)
        data[i] =  m_biped->getPDControllers()[i-7]->getKpGain();

    /// ========================================== ///

}

void Application::Step(Settings* settings) {

	double currentTime = glfwGetTime();
	double timeStep = currentTime - m_lastUpdate;
	m_lastUpdate = currentTime;

	if (settings->pause) {
		if (settings->singleStep) settings->singleStep = 0;
		else timeStep = 0.0f;
		g_debugDraw.DrawString(522, m_textLine, "****PAUSE****");
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	g_debugDraw.SetFlags(b2Draw::e_shapeBit + settings->drawJoints * b2Draw::e_jointBit);
	m_pointCount = 0;

	// Lancer la balle
	if (settings->launchBalls) {
        double time = glfwGetTime();
	    // Lancement d'une balle toutes les 3 secondes
	    if (time - m_lastShoot > 3.0f) {
	        m_lastShoot = time;
	        LaunchBall();
	    }
	}

	if (settings->steps) {
	    // creer des nouvelles marches
	    bool needNewStepBoxes = ((((int)(m_biped->getCOM().x * 1.0f)) % 5) == 0) && m_steps.empty();
	    if (needNewStepBoxes) {
            int nbSteps = (rand() % 5) + 1;
            for (int i=0;i<nbSteps;i++) {
                float longeurMarche = RandomFloat(0.2,1.0);
                float hauteurMarche = RandomFloat(0.01,0.05);
                b2BodyDef bd;
                bd.type = b2_staticBody;
                if (i==0) bd.position = b2Vec2(m_biped->getCOM().x + 2.0,hauteurMarche/2.0);
                else bd.position = b2Vec2(m_steps[i-1]->GetPosition().x + m_steps_size[i-1]/2.0 + longeurMarche/2.0,hauteurMarche/2.0);
                b2Body* newStep = m_world->CreateBody(&bd);
                b2PolygonShape shape;
                shape.SetAsBox(longeurMarche/2.0,hauteurMarche/2.0);
                b2FixtureDef fixture;
                fixture.shape = &shape;
                fixture.density = 5.0f;
                fixture.friction = 0.999;
                newStep->CreateFixture(&fixture);
                m_steps.push_back(newStep);
                m_steps_size.push_back(longeurMarche);
            }
	    }
	    // supprimer d'anciennes marches
	    if (!m_steps.empty() && m_steps[m_steps.size()-1]->GetPosition().x < m_biped->getCOM().x - 2.0) {
            for (auto it = m_steps.begin();it!=m_steps.end();it++) m_world->DestroyBody(*it);
            m_steps.clear();
            m_steps_size.clear();
	    }
	}

	if (!settings->pause) {

	    // Simule un pas de temps
    	m_world->Step(timeStep, 8, 3);

    	// Test points de contact pieds/sol
	    bool leftFootInContact = false;
	    bool rightFootInContact = false;
	    for (int32 i = 0; i < m_pointCount; ++i) {
			ContactPoint* point = m_points + i;
			if (point->state == b2_addState || point->state == b2_persistState) {
			    if ( ((int)point->fixtureA->GetUserData() == ID_GROUND && (int)point->fixtureB->GetUserData() == 0)  || // 0 = ID PIED_GAUCHE
                     ((int)point->fixtureB->GetUserData() == ID_GROUND && (int)point->fixtureA->GetUserData() == 0)   )
                     leftFootInContact = true;

       		    if ( ((int)point->fixtureA->GetUserData() == ID_GROUND && (int)point->fixtureB->GetUserData() == 1)  || // 1 = ID PIED_GAUCHE
                     ((int)point->fixtureB->GetUserData() == ID_GROUND && (int)point->fixtureA->GetUserData() == 1)   )
                     rightFootInContact = true;

			}
	    }

	    // Controle du bipede
	    m_biped->update(timeStep,leftFootInContact,rightFootInContact);

	}

	// Visualiser le CdM
	if (settings->showCOM) {
        if (m_COM == NULL) {
            // on cree le CdM
            b2BodyDef bd;
            bd.position = m_biped->getCOM();
            bd.allowSleep = false;
            bd.awake = true;
            bd.active = false;
            m_COM = m_world->CreateBody(&bd);
            b2CircleShape circle;
            circle.m_radius = 0.05f;
            m_COM->CreateFixture(&circle, 0.0f);
        }
        else {
            m_COM->SetTransform(m_biped->getCOM(),0.0f);
        }
	}
	else {
	    if (m_COM) {
            // on detruit le CdM
            m_world->DestroyBody(m_COM);
			m_COM = NULL;
	    }
	}

	// Deplacer la camera pour suivre le personnage
	g_camera.m_center.x = m_biped->getCOM().x;

	m_world->DrawDebugData();
    g_debugDraw.Flush();

   	// Affichage de la distance parcourue et des bornes
   	float COM_x = ((int)(m_biped->getCOM().x * 100.0f))/100.0f;
   	if (!m_biped->hasFallen()) m_walkDistance = COM_x;
   	std::ostringstream osstmp;
   	osstmp << "Distance parcourue : " << m_walkDistance << " metres";
   	g_debugDraw.DrawString(5,30,osstmp.str().c_str());
   	int i_COM_x = (int) COM_x;
   	for (int i = i_COM_x - 2; i <= i_COM_x + 2; i++) {
   	    std::ostringstream borne; borne << i;
   	    b2Vec2 posBorne = g_camera.ConvertWorldToScreen(b2Vec2(i,0.0));
       	g_debugDraw.DrawString(posBorne.x,730,borne.str().c_str());
   	}

	if (timeStep > 0.0f) ++m_stepCount;

	// Track maximum profile times
	{
		const b2Profile& p = m_world->GetProfile();
		m_maxProfile.step = b2Max(m_maxProfile.step, p.step);
		m_maxProfile.collide = b2Max(m_maxProfile.collide, p.collide);
		m_maxProfile.solve = b2Max(m_maxProfile.solve, p.solve);
		m_maxProfile.solveInit = b2Max(m_maxProfile.solveInit, p.solveInit);
		m_maxProfile.solveVelocity = b2Max(m_maxProfile.solveVelocity, p.solveVelocity);
		m_maxProfile.solvePosition = b2Max(m_maxProfile.solvePosition, p.solvePosition);
		m_maxProfile.solveTOI = b2Max(m_maxProfile.solveTOI, p.solveTOI);
		m_maxProfile.broadphase = b2Max(m_maxProfile.broadphase, p.broadphase);

		m_totalProfile.step += p.step;
		m_totalProfile.collide += p.collide;
		m_totalProfile.solve += p.solve;
		m_totalProfile.solveInit += p.solveInit;
		m_totalProfile.solveVelocity += p.solveVelocity;
		m_totalProfile.solvePosition += p.solvePosition;
		m_totalProfile.solveTOI += p.solveTOI;
		m_totalProfile.broadphase += p.broadphase;
	}

	if (m_mouseJoint) {
		b2Vec2 p1 = m_mouseJoint->GetAnchorB();
		b2Vec2 p2 = m_mouseJoint->GetTarget();

		b2Color c;
		c.Set(0.0f, 1.0f, 0.0f);
		g_debugDraw.DrawPoint(p1, 4.0f, c);
		g_debugDraw.DrawPoint(p2, 4.0f, c);

		c.Set(0.8f, 0.8f, 0.8f);
		g_debugDraw.DrawSegment(p1, p2, c);
	}

	if (settings->drawContactPoints) {
		const float32 k_axisScale = 0.3f;
		for (int32 i = 0; i < m_pointCount; ++i) {
			ContactPoint* point = m_points + i;
			if (point->state == b2_addState) {
				// Add
				g_debugDraw.DrawPoint(point->position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
			}
			else if (point->state == b2_persistState) {
				// Persist
				g_debugDraw.DrawPoint(point->position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
			}

			if (settings->drawContactNormals == 1) {
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = p1 + k_axisScale * point->normal;
				g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
			}
		}
	}
}

void Application::ShiftOrigin(const b2Vec2& newOrigin) {
	m_world->ShiftOrigin(newOrigin);
}
