
#include <GameObject/GOScore.h>
#include <WallBall.h>


// TODO Reverse engineered because missing. Displays the score?

GOScore::GOScore(int playerNum) : GameObject() {
  this->m_playernum = playerNum;
}

GOScore::~GOScore() {}

void GOScore::updateScore(int score) {
  this->m_score = score;
}
