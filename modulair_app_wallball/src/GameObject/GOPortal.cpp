
#include <GameObject/GOPortal.h>
#include <Components/CController.h>
#include <Components/CPhysicsObject.h>
#include <Box2D/Box2D.h>

#include <WallBall.h>

void GOPortal::setupPortal(float x1, float y1, float x2, float y2, const char* matFile) {
	if (x1 > x2) {
		float temp = x1;
		x1 = x2;
		x2 = temp;
		temp = y1;
		y1 = y2;
		y2 = temp;
	}


	GOPortal* portal1;
	GOPortal* portal2;
	
    if (matFile == 0) {
        portal1 = new GOPortal(x1, y1, 3.0f);
        portal2 = new GOPortal(x2, y2, -3.0f);
    }
    else {
        portal1 = new GOPortal(x1, y1, 3.0f, matFile);
        portal2 = new GOPortal(x2, y2, -3.0f, matFile);
    }

	portal1->setTarget(portal2);
	portal2->setTarget(portal1);
}

GOPortal::GOPortal(float x, float y, float direction) :
    m_X(x), m_Y(y),
	GOBlock(x, y, 0.5f, 5.5f, 0.0f, (WallBall::s_AssetPath + "/color3.bmp").c_str()),
	m_Direction(direction),
	m_Target(0) {
}

GOPortal::GOPortal(float x, float y, float direction, const char* matFile) :
    m_X(x), m_Y(y),
	GOBlock(x, y, 0.5f, 5.5f, 0.0f, matFile),
	m_Direction(direction),
	m_Target(0) {
}

GOPortal::~GOPortal() {
	
}

void GOPortal::setTarget(GOPortal* portal) {
	m_Target = portal;
}

void GOPortal::onCollision(GameObject* other) {

	// if the other GameObject is a player and is moving into this portal, teleport it
	if (other->getComponent(CController::classTypeID()) != 0) {
		b2Body* body = ((CPhysicsObject *)other->getComponent(CPhysicsObject::classTypeID()))->getBody();
		b2Vec2 velocity = body->GetLinearVelocity();
		
		// if the player is moving into portal, teleport it
		if (velocity.x * m_Direction > 0.0f) {
			b2Body* targetBody = ((CPhysicsObject *)m_Target->getComponent(CPhysicsObject::classTypeID()))->getBody();
            float yDiff = m_Y - body->GetPosition().y;
			b2Vec2 targetPosition = targetBody->GetPosition();
			other->setNewPoint(targetPosition + b2Vec2(-m_Direction, 0.0f) - b2Vec2(0.0f, yDiff));
		}
	}
}
