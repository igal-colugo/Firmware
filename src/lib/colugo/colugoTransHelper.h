/**
* @file colugoTransHelper.h
helper file for trasitioning f
*/

//#ifndef COLUGOTRANSITIONHELPER_H
//#define COLUGOTRANSITIONHELPER_H

#pragma once

class colugoTransHelper
{
private:
	/* data */
	enum class TRANSITION_TO_FW_STATE {
		NOT_IN_TRANSITION = 0,
		IN_TRANSITION,
		PAST_BLEND_SPEED,
		PAST_TRANS_SPEED
	};

	TRANSITION_TO_FW_STATE _transStage = TRANSITION_TO_FW_STATE::NOT_IN_TRANSITION;
public:
	colugoTransHelper(/* args */);
	~colugoTransHelper() = default;
};

//#endif
