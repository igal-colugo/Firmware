
/**
 * @file colugoCamGuide.h
 *
 * Helper class to colugo camera guide
 *
 * @author igal gosis
 */

#pragma once

#include "navigator_mode.h"
#include "mission_block.h"

#include <px4_platform_common/module_params.h>

class ColugoCamGuide : public MissionBlock, public ModuleParams
{
public:
	ColugoCamGuide(Navigator *navigator);
	~ColugoCamGuide() = default;

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

private:
	/**
	 * Use the stored reposition location of the navigator
	 * to move to a new location.
	 */
	void reposition();

	/**
	 * Set the position to hold based on the current local position
	 */
	void set_cam_guide_position();

	bool _cam_guide_pos_set{false};
};
