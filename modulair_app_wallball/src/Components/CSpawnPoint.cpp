
#include <Components/CSpawnPoint.h>
#include <vector>
#include <stdlib.h>
#include <GameObject/GameObject.h>

std::vector<CSpawnPoint*> CSpawnPoint::s_SpawnPoints = std::vector<CSpawnPoint*>();

bool CSpawnPoint::s_HasSeeded = false;
CSpawnPoint* CSpawnPoint::getRandomSpawnPoint() {
	if (!CSpawnPoint::s_HasSeeded) {
		//srand(time(NULL));
		CSpawnPoint::s_HasSeeded = true;
	}
	
	int size = s_SpawnPoints.size();
	if (size > 0)
		return s_SpawnPoints[rand() % size];
	return 0;
}

CSpawnPoint::CSpawnPoint(GameObject* gameObject, float x, float y) :
	Component(gameObject),
	m_X(x), m_Y(y)
{
	s_SpawnPoints.push_back(this);
}

CSpawnPoint::~CSpawnPoint() {
	
}

int CSpawnPoint::s_TypeID = -1;
int CSpawnPoint::componentTypeID() {
	return classTypeID();
}

int CSpawnPoint::classTypeID() {
	if (s_TypeID < 0)
		s_TypeID = Component::nextTypeID();
	return s_TypeID;
}

void CSpawnPoint::start() {
	
}

void CSpawnPoint::tick() {
	
}

void CSpawnPoint::move(float dX, float dY) {
	m_X += dX;
	m_Y += dY;
}

float CSpawnPoint::getX() {
	return m_X;
}

float CSpawnPoint::getY() {
	return m_Y;
}
