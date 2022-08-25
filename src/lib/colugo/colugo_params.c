/**
 * @file colugo_params.c
 *
 * Proprietory Parameters for colugo.
 *
 * @author igal gosis
 */

/**
 * Wing Lock First Pos
 *
 * set the first position in trasition form MC to FW
 * of the wing lock actuator
 *
 * @group Colugo
 * @unit norm
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(C_WAFP, 0.2f);
