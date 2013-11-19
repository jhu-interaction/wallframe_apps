
#ifndef GO_SCORE
#define GO_SCORE

#include <GameObject/GameObject.h>

// Reverse engineered since it is missing. Inherit from GameObject?

class GOScore : public GameObject {
	
public:

	GOScore(int playerNum);
	~GOScore();
	void updateScore(int score);

private:
	int m_playernum;
	int m_score;
};

#endif
