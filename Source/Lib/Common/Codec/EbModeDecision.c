/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

/*
* Copyright (c) 2016, Alliance for Open Media. All rights reserved
*
* This source code is subject to the terms of the BSD 2 Clause License and
* the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
* was not distributed with this source code in the LICENSE file, you can
* obtain it at www.aomedia.org/license/software. If the Alliance for Open
* Media Patent License 1.0 was not distributed with this source code in the
* PATENTS file, you can obtain it at www.aomedia.org/license/patent.
*/

/***************************************
* Includes
***************************************/
#include <stdlib.h>

#include "EbDefinitions.h"
#include "EbUtility.h"
#include "EbSequenceControlSet.h"

#include "EbModeDecision.h"
#include "EbAdaptiveMotionVectorPrediction.h"
#include "EbTransformUnit.h"
#include "EbModeDecisionProcess.h"
#include "EbMotionEstimation.h"
#include "aom_dsp_rtcd.h"

#include "av1me.h"
#include "hash.h"
#if COMP_DIFF
#include "EbInterPrediction.h"
#endif
#if II_COMP
#include "aom_dsp_rtcd.h"
#include "EbRateDistortionCost.h"
#endif
#if FI_MD
int av1_filter_intra_allowed_bsize(uint8_t enable_filter_intra, BlockSize bs);
#endif
#if CHECK_CAND
#define  INCRMENT_CAND_TOTAL_COUNT(cnt) cnt++; if(cnt>=MODE_DECISION_CANDIDATE_MAX_COUNT) printf(" ERROR: reaching limit for MODE_DECISION_CANDIDATE_MAX_COUNT %i\n",cnt);
#endif
int8_t av1_ref_frame_type(const MvReferenceFrame *const rf);
#if II_COMP
void av1_set_ref_frame(MvReferenceFrame *rf,
    int8_t ref_frame_type);


static INLINE int is_interintra_allowed_bsize(const BLOCK_SIZE bsize) {
    return (bsize >= BLOCK_8X8) && (bsize <= BLOCK_32X32);
}

static INLINE int is_interintra_allowed_mode(const PredictionMode mode) {
    return (mode >= SINGLE_INTER_MODE_START) && (mode < SINGLE_INTER_MODE_END);
}

static INLINE int is_interintra_allowed_ref(const MvReferenceFrame rf[2]) {
    return (rf[0] > INTRA_FRAME) && (rf[1] <= INTRA_FRAME);
}

static INLINE int is_interintra_allowed(const MB_MODE_INFO *mbmi) {
  return is_interintra_allowed_bsize(mbmi->sb_type) &&
         is_interintra_allowed_mode(mbmi->mode) &&
         is_interintra_allowed_ref(mbmi->ref_frame);
}



int svt_is_interintra_allowed(
    uint8_t enable_inter_intra,
    BlockSize sb_type,
    PredictionMode mode,
    MvReferenceFrame ref_frame[2])
{
    return
        enable_inter_intra &&
        is_interintra_allowed_bsize((const BlockSize)sb_type) &&
        is_interintra_allowed_mode(mode)  &&
        is_interintra_allowed_ref(ref_frame);

}
#endif
/********************************************
* Constants
********************************************/
#if II_SEARCH
// 1 - Regular uni-pred ,
// 2 - Regular uni-pred + Wedge compound Inter Intra
// 3 - Regular uni-pred + Wedge compound Inter Intra + Smooth compound Inter Intra

#define II_COUNT                3
#endif
#if COMP_MODE

static int32_t have_newmv_in_inter_mode(PredictionMode mode) {
    return (mode == NEWMV || mode == NEW_NEWMV || mode == NEAREST_NEWMV ||
        mode == NEW_NEARESTMV || mode == NEAR_NEWMV || mode == NEW_NEARMV);
}
#endif
//static uint32_t  AntiContouringIntraMode[11] = { EB_INTRA_PLANAR, EB_INTRA_DC, EB_INTRA_HORIZONTAL, EB_INTRA_VERTICAL,
//EB_INTRA_MODE_2, EB_INTRA_MODE_6, EB_INTRA_MODE_14, EB_INTRA_MODE_18, EB_INTRA_MODE_22, EB_INTRA_MODE_30, EB_INTRA_MODE_34 };

const uint32_t parentIndex[85] = { 0, 0, 0, 2, 2, 2, 2, 0, 7, 7, 7, 7, 0, 12, 12, 12, 12, 0, 17, 17, 17, 17, 0, 0,
23, 23, 23, 23, 0, 28, 28, 28, 28, 0, 33, 33, 33, 33, 0, 38, 38, 38, 38, 0, 0,
44, 44, 44, 44, 0, 49, 49, 49, 49, 0, 54, 54, 54, 54, 0, 59, 59, 59, 59, 0, 0,
65, 65, 65, 65, 0, 70, 70, 70, 70, 0, 75, 75, 75, 75, 0, 80, 80, 80, 80 };
#if MRP_LIST_REF_IDX_TYPE_LT
/*
  NORMAL ORDER
  |-------------------------------------------------------------|
  | ref_idx          0            1           2            3       |
  | List0            LAST        LAST2        LAST3        GOLD    |
  | List1            BWD            ALT2        ALT                 |
  |-------------------------------------------------------------|
*/
#define INVALID_REF 0xF

#if INJ_MVP
uint8_t get_list_idx(uint8_t ref_type) {
    if (ref_type == LAST_FRAME || ref_type == LAST2_FRAME || ref_type == LAST3_FRAME || ref_type == GOLDEN_FRAME)
        return 0;
    else if (ref_type == BWDREF_FRAME || ref_type == ALTREF_FRAME || ref_type == ALTREF2_FRAME)
        return 1;
    else
        return (INVALID_REF);
};
#endif

#if  MCP_4XN_FIX
uint8_t get_ref_frame_idx(uint8_t ref_type) {
#if NORMAL_ORDER
    if (ref_type == LAST_FRAME || ref_type == BWDREF_FRAME)
        return 0;
    else if (ref_type == LAST2_FRAME || ref_type == ALTREF2_FRAME)
        return 1;
    else if (ref_type == LAST3_FRAME || ref_type == ALTREF_FRAME)
        return 2;
    else if (ref_type == GOLDEN_FRAME)
        return 3;
    else
        return (INVALID_REF);
#else
    if (ref_type == LAST_FRAME || ref_type == BWDREF_FRAME)
        return 0;
    else if (ref_type == LAST2_FRAME || ref_type == ALTREF_FRAME)
        return 1;
    else if (ref_type == LAST3_FRAME || ref_type == ALTREF2_FRAME)
        return 2;
    else if (ref_type == GOLDEN_FRAME)
        return 3;
    else
        return (INVALID_REF);
#endif
};
#else
uint8_t get_ref_frame_idx(uint8_t list, uint8_t ref_type) {
    switch (list) {
    case 0:
        return (ref_type == LAST_FRAME ? 0 : ref_type == LAST2_FRAME ? 1 : ref_type == LAST3_FRAME ? 2 : ref_type == GOLDEN_FRAME ? 3 : INVALID_REF);
    case 1:
        return (ref_type == BWDREF_FRAME ? 0 : ref_type == ALTREF_FRAME ? 1 : ref_type == ALTREF2_FRAME ? 2 : INVALID_REF);
    default:
        return (INVALID_REF);
    }
};
#endif
MvReferenceFrame svt_get_ref_frame_type(uint8_t list, uint8_t ref_idx) {
#if NORMAL_ORDER
    switch (list) {
    case 0:
        return (ref_idx == 0 ? LAST_FRAME : ref_idx == 1 ? LAST2_FRAME : ref_idx == 2 ? LAST3_FRAME : ref_idx == 3 ? GOLDEN_FRAME : INVALID_REF);
    case 1:
        return (ref_idx == 0 ? BWDREF_FRAME : ref_idx == 1 ? ALTREF2_FRAME : ref_idx == 2 ? ALTREF_FRAME : INVALID_REF);
    default:
        return (INVALID_REF);
    }
#else
    switch (list) {
    case 0:
        return (ref_idx == 0 ? LAST_FRAME : ref_idx == 1 ? LAST2_FRAME : ref_idx == 2 ? LAST3_FRAME : ref_idx == 3 ? GOLDEN_FRAME : INVALID_REF);
    case 1:
        return (ref_idx == 0 ? BWDREF_FRAME : ref_idx == 1 ? ALTREF_FRAME : ref_idx == 2 ? ALTREF2_FRAME : INVALID_REF);
    default:
        return (INVALID_REF);
    }
#endif
};
#endif
extern uint32_t stage1ModesArray[];

uint8_t GetMaxDrlIndex(uint8_t  refmvCnt, PredictionMode   mode);
int32_t av1_mv_bit_cost(const MV *mv, const MV *ref, const int32_t *mvjcost,
    int32_t *mvcost[2], int32_t weight);
#define MV_COST_WEIGHT 108

#if II_COMP
#define MAX_INTERINTRA_SB_SQUARE 32 * 32
 int is_interintra_wedge_used(BLOCK_SIZE sb_type) {
    return wedge_params_lookup[sb_type].bits > 0;
}

EbErrorType  intra_luma_prediction_for_interintra(
    ModeDecisionContext         *md_context_ptr,
    PictureControlSet           *picture_control_set_ptr,
    INTERINTRA_MODE              interintra_mode,
    EbPictureBufferDesc         *prediction_ptr);
int64_t pick_wedge_fixed_sign(
    ModeDecisionCandidate        *candidate_ptr,
    PictureControlSet                    *picture_control_set_ptr,
    ModeDecisionContext                  *context_ptr,
    const BLOCK_SIZE bsize,
    const int16_t *const residual1,
    const int16_t *const diff10,
    const int8_t wedge_sign,
    int8_t *const best_wedge_index);
void model_rd_for_sb_with_curvfit(
    PictureControlSet      *picture_control_set_ptr,
    ModeDecisionContext                  *context_ptr,
    BLOCK_SIZE bsize, int bw, int bh,
    uint8_t* src_buf, uint32_t src_stride, uint8_t* pred_buf, uint32_t pred_stride,
    int plane_from, int plane_to, int mi_row, int mi_col, int *out_rate_sum,
    int64_t *out_dist_sum, int *skip_txfm_sb, int64_t *skip_sse_sb,
    int *plane_rate, int64_t *plane_sse, int64_t *plane_dist);
static int64_t pick_interintra_wedge(
    ModeDecisionCandidate        *candidate_ptr,
    //const AV1_COMP *const cpi,
    //const MACROBLOCK *const x,
    PictureControlSet                    *picture_control_set_ptr,
    ModeDecisionContext                  *context_ptr,
    const BLOCK_SIZE bsize,
    const uint8_t *const p0,
    const uint8_t *const p1,
    uint8_t * src_buf,
    uint32_t  src_stride,
    int32_t *wedge_index_out
    )
{
    //const MACROBLOCKD *const xd = &x->e_mbd;
    //MB_MODE_INFO *const mbmi = xd->mi[0];
    assert(is_interintra_wedge_used(bsize));
   // assert(cpi->common.seq_params.enable_interintra_compound);

   // const struct buf_2d *const src = &x->plane[0].src;
    const int bw = block_size_wide[bsize];
    const int bh = block_size_high[bsize];
    DECLARE_ALIGNED(32, int16_t, residual1[MAX_SB_SQUARE]);  // src - pred1
    DECLARE_ALIGNED(32, int16_t, diff10[MAX_SB_SQUARE]);     // pred1 - pred0
    //if (is_cur_buf_hbd(xd)) {
    //    aom_highbd_subtract_block(bh, bw, residual1, bw, src->buf, src->stride,
    //        CONVERT_TO_BYTEPTR(p1), bw, xd->bd);
    //    aom_highbd_subtract_block(bh, bw, diff10, bw, CONVERT_TO_BYTEPTR(p1), bw,
    //        CONVERT_TO_BYTEPTR(p0), bw, xd->bd);
    //}
    //else
    {

        //int rows, int cols, int16_t *diff,
        //    ptrdiff_t diff_stride, const uint8_t *src,
        //    ptrdiff_t src_stride, const uint8_t *pred,
        //    ptrdiff_t pred_stride


        aom_subtract_block(bh, bw, residual1, bw, src_buf, src_stride, p1, bw);
        aom_subtract_block(bh, bw, diff10, bw, p1, bw, p0, bw);
    }

    int8_t /*int*/ wedge_index = -1;
    int64_t rd =
        pick_wedge_fixed_sign(candidate_ptr,picture_control_set_ptr, context_ptr, bsize, residual1, diff10, 0, &wedge_index);

    *wedge_index_out = wedge_index;

   // mbmi->interintra_wedge_sign = 0;
   // mbmi->interintra_wedge_index = wedge_index;
    return rd;
}
#if II_SO
//for every CU, perform DC/V/H/S intra prediction to be used later in inter-intra search
void precompute_intra_pred_for_inter_intra(
    PictureControlSet            *picture_control_set_ptr,
    ModeDecisionContext          *context_ptr)
{
    uint32_t j;
    EbPictureBufferDesc  pred_desc;
    pred_desc.origin_x = pred_desc.origin_y = 0;
    pred_desc.stride_y = context_ptr->blk_geom->bwidth;

    for (j = 0; j < INTERINTRA_MODES; ++j)
    {

        INTERINTRA_MODE interintra_mode = (INTERINTRA_MODE)j;
        pred_desc.buffer_y = context_ptr->intrapred_buf[j];
        intra_luma_prediction_for_interintra(
            context_ptr,
            picture_control_set_ptr,
            interintra_mode,
            &pred_desc);
    }
}
#endif
 void combine_interintra(INTERINTRA_MODE mode,
    int8_t use_wedge_interintra, int wedge_index,
    int wedge_sign, BLOCK_SIZE bsize,
    BLOCK_SIZE plane_bsize, uint8_t *comppred,
    int compstride, const uint8_t *interpred,
    int interstride, const uint8_t *intrapred,
    int intrastride);
void inter_intra_search(
    PictureControlSet            *picture_control_set_ptr,
    ModeDecisionContext          *context_ptr,
    ModeDecisionCandidate        *candidate_ptr)
{
    DECLARE_ALIGNED(16, uint8_t, tmp_buf[ MAX_INTERINTRA_SB_SQUARE]);
    DECLARE_ALIGNED(16, uint8_t, intrapred[ MAX_INTERINTRA_SB_SQUARE]);

    DECLARE_ALIGNED(16, uint8_t, ii_pred_buf[MAX_INTERINTRA_SB_SQUARE]);
    //get inter pred for ref0
    EbPictureBufferDesc   *src_pic = picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr;
    uint8_t               *src_buf = src_pic->buffer_y + (context_ptr->cu_origin_x + src_pic->origin_x) + (context_ptr->cu_origin_y + src_pic->origin_y) * src_pic->stride_y;

    uint32_t  bwidth = context_ptr->blk_geom->bwidth;
    uint32_t  bheight = context_ptr->blk_geom->bheight;
    EbPictureBufferDesc  pred_desc;
    pred_desc.origin_x = pred_desc.origin_y = 0;
    pred_desc.stride_y = bwidth;

    SequenceControlSet* sequence_control_set_ptr = ((SequenceControlSet*)(picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr));
    EbPictureBufferDesc  *ref_pic_list0;
    EbPictureBufferDesc  *ref_pic_list1 = NULL;
    Mv mv_0;
    Mv mv_1;
    mv_0.x = candidate_ptr->motion_vector_xl0;
    mv_0.y = candidate_ptr->motion_vector_yl0;
    mv_1.x = candidate_ptr->motion_vector_xl1;
    mv_1.y = candidate_ptr->motion_vector_yl1;
    MvUnit mv_unit;
    mv_unit.mv[0] = mv_0;
    mv_unit.mv[1] = mv_1;
    int8_t ref_idx_l0 = candidate_ptr->ref_frame_index_l0;
    int8_t ref_idx_l1 = candidate_ptr->ref_frame_index_l1;
    MvReferenceFrame rf[2];
    av1_set_ref_frame(rf, candidate_ptr->ref_frame_type);
    uint8_t list_idx0, list_idx1;
    list_idx0 = get_list_idx(rf[0]);
    if (rf[1] == NONE_FRAME)
        list_idx1 = get_list_idx(rf[0]);
    else
        list_idx1 = get_list_idx(rf[1]);
    assert(list_idx0 < MAX_NUM_OF_REF_PIC_LIST);
    assert(list_idx1 < MAX_NUM_OF_REF_PIC_LIST);
    if (ref_idx_l0 >= 0)
        ref_pic_list0 = ((EbReferenceObject*)picture_control_set_ptr->ref_pic_ptr_array[list_idx0][ref_idx_l0]->object_ptr)->reference_picture;
    else
        ref_pic_list0 = (EbPictureBufferDesc*)EB_NULL;
    if (ref_idx_l1 >= 0)
        ref_pic_list1 = ((EbReferenceObject*)picture_control_set_ptr->ref_pic_ptr_array[list_idx1][ref_idx_l1]->object_ptr)->reference_picture;
    else
        ref_pic_list1 = (EbPictureBufferDesc*)EB_NULL;

    mv_unit.pred_direction = candidate_ptr->prediction_direction[0];

    pred_desc.buffer_y = tmp_buf;

    //int64_t skip_sse_sb = INT64_MAX;
    //int32_t skip_txfm_sb = 0;
    //int32_t rs = 0;
    //int64_t rd = INT64_MAX;


    //interpolation_filter_search(
    //                picture_control_set_ptr,
    //                &pred_desc, //output,
    //                context_ptr,
    //                candidate_buffer_ptr,
    //                mv_unit,
    //                ref_pic_list0,
    //                ref_pic_list1,
    //                sequence_control_set_ptr->encode_context_ptr->asm_type,
    //                &rd,
    //                &rs,
    //                &skip_txfm_sb,
    //                &skip_sse_sb);




    //we call the regular inter prediction path here(no compound)
    av1_inter_prediction(
        picture_control_set_ptr,
        0,//ASSUMPTION: fixed interpolation filter.
        context_ptr->cu_ptr,
        candidate_ptr->ref_frame_type,
        &mv_unit,
        0,//use_intrabc,
#if COMP_MODE
        1,//compound_idx not used
#endif
#if COMP_DIFF
        NULL,// interinter_comp not used
#endif
#if II_ED
        NULL,
        NULL,
        NULL,
        NULL,
        0,
        0,
        0,
        0,
#endif
        context_ptr->cu_origin_x,
        context_ptr->cu_origin_y,
        bwidth,
        bheight,
        ref_pic_list0,
        ref_pic_list1,
        &pred_desc, //output
        0,          //output origin_x,
        0,          //output origin_y,
        0 //do chroma
        );


    const int is_wedge_used = is_interintra_wedge_used(context_ptr->blk_geom->bsize);
    assert(is_wedge_used);//if not I need to add nowedge path!!

    int64_t best_interintra_rd_wedge = INT64_MAX;
    int64_t rd = INT64_MAX;
    int64_t best_interintra_rd = INT64_MAX;
    int rmode = 0, rate_sum;
    int64_t dist_sum;
    int tmp_rate_mv = 0;


    INTERINTRA_MODE best_interintra_mode = INTERINTRA_MODES;
    EbPictureBufferDesc  inra_pred_desc;
    inra_pred_desc.origin_x = inra_pred_desc.origin_y = 0;
    inra_pred_desc.stride_y = bwidth;
    inra_pred_desc.buffer_y = intrapred;

    int8_t enable_smooth_interintra =1;
      //if (cpi->oxcf.enable_smooth_interintra &&
      //!cpi->sf.disable_smooth_interintra) {
    if (enable_smooth_interintra) {
        int j = 0;
        if (/*cpi->sf.reuse_inter_intra_mode == 0 ||*/
            best_interintra_mode == INTERINTRA_MODES) {
            for (j = 0; j < INTERINTRA_MODES; ++j) {
                //if ((!cpi->oxcf.enable_smooth_intra || cpi->sf.disable_smooth_intra) &&
                //    (INTERINTRA_MODE)j == II_SMOOTH_PRED)
                //  continue;
                INTERINTRA_MODE interintra_mode = (INTERINTRA_MODE)j;
                //rmode = interintra_mode_cost[mbmi->interintra_mode];
                const int bsize_group = size_group_lookup[context_ptr->blk_geom->bsize];
                rmode  = candidate_ptr->md_rate_estimation_ptr->inter_intra_mode_fac_bits[bsize_group][interintra_mode];
#if !II_SO
                inra_pred_desc.buffer_y = context_ptr->intrapred_buf[j];

                //av1_build_intra_predictors_for_interintra(cm, xd, bsize, 0, orig_dst,
                //                                          intrapred, bw);
                intra_luma_prediction_for_interintra(
                    context_ptr,
                    picture_control_set_ptr,
                    interintra_mode,
                    &inra_pred_desc);
#endif

                //av1_combine_interintra(xd, bsize, 0, tmp_buf, bw, intrapred, bw);
                combine_interintra(
                    interintra_mode,//mode,
                    0,//use_wedge_interintra,
                    0,//candidate_ptr->interintra_wedge_index,
                    0,//int wedge_sign,
                    context_ptr->blk_geom->bsize,
                    context_ptr->blk_geom->bsize,// plane_bsize,
                    ii_pred_buf, bwidth, /*uint8_t *comppred, int compstride,*/
                    tmp_buf, bwidth,  /*const uint8_t *interpred, int interstride,*/
#if II_SO
                    context_ptr->intrapred_buf[j], bwidth /*const uint8_t *intrapred,   int intrastride*/);
#else
                    intrapred, bwidth /*const uint8_t *intrapred,   int intrastride*/);
#endif

                //model_rd_sb_fn[MODELRD_TYPE_INTERINTRA](
                //    cpi, bsize, x, xd, 0, 0, mi_row, mi_col, &rate_sum, &dist_sum,
                //    &tmp_skip_txfm_sb, &tmp_skip_sse_sb, NULL, NULL, NULL);
                model_rd_for_sb_with_curvfit(picture_control_set_ptr, context_ptr, context_ptr->blk_geom->bsize, bwidth, bheight,
                    src_buf, src_pic->stride_y, ii_pred_buf, bwidth,
                    0, 0, 0, 0, &rate_sum, &dist_sum, NULL, NULL, NULL, NULL, NULL);

                // rd = RDCOST(x->rdmult, tmp_rate_mv + rate_sum + rmode, dist_sum);
                rd = RDCOST(context_ptr->full_lambda, tmp_rate_mv + rate_sum + rmode, dist_sum);

                if (rd < best_interintra_rd) {
                    best_interintra_rd = rd;
                    candidate_ptr->interintra_mode = best_interintra_mode = interintra_mode;
                }
            }


#if !  II_SO
            intra_luma_prediction_for_interintra(
                context_ptr,
                picture_control_set_ptr,
                best_interintra_mode,
                &inra_pred_desc);
#endif


            /* best_interintra_rd_wedge =
                 pick_interintra_wedge(cpi, x, bsize, intrapred_, tmp_buf_);*/

            //CHKN need to re-do intra pred using the winner, or have a separate intra serch for wedge

            best_interintra_rd_wedge = pick_interintra_wedge(
                candidate_ptr,
                picture_control_set_ptr,
                context_ptr,
                context_ptr->blk_geom->bsize,
#if II_SO
                context_ptr->intrapred_buf[best_interintra_mode],
#else
                intrapred,
#endif
                tmp_buf,
                src_buf,
                src_pic->stride_y,
                &candidate_ptr->interintra_wedge_index
            );

            //if (best_interintra_rd_wedge < best_interintra_rd) {

                //candidate_ptr->use_wedge_interintra = 1;
                //candidate_ptr->ii_wedge_sign = 0;
            //}
            //args->inter_intra_mode[mbmi->ref_frame[0]] = best_interintra_mode;
        }
    }
    // Enable wedge search if source variance and edge strength are above the thresholds.
    int enable_wedge_interintra_search = 0;
    if (enable_wedge_interintra_search)
    {
        if (best_interintra_mode == INTERINTRA_MODES) {//Optimization TBD: search only for the first MV mode per ref

            INTERINTRA_MODE   interintra_mode = II_SMOOTH_PRED;
            intra_luma_prediction_for_interintra(
                context_ptr,
                picture_control_set_ptr,
                interintra_mode,
                &inra_pred_desc);

            candidate_ptr->ii_wedge_sign = 0;

            pick_interintra_wedge(
                candidate_ptr,
                picture_control_set_ptr,
                context_ptr,
                context_ptr->blk_geom->bsize,
                intrapred,
                tmp_buf,
                src_buf,
                src_pic->stride_y,
                &candidate_ptr->interintra_wedge_index
                );

            int j = 0;
            for (j = 0; j < INTERINTRA_MODES; ++j) {

                interintra_mode = (INTERINTRA_MODE)j;

                const int bsize_group = size_group_lookup[context_ptr->blk_geom->bsize];
                rmode  = candidate_ptr->md_rate_estimation_ptr->inter_intra_mode_fac_bits[bsize_group][interintra_mode];

                intra_luma_prediction_for_interintra(
                    context_ptr,
                    picture_control_set_ptr,
                    interintra_mode,
                    &inra_pred_desc);

                combine_interintra(
                    interintra_mode,//mode,
                    1,//use_wedge_interintra,
                    candidate_ptr->interintra_wedge_index,
                    0,//int wedge_sign,
                    context_ptr->blk_geom->bsize,
                    context_ptr->blk_geom->bsize,// plane_bsize,
                    ii_pred_buf, bwidth, /*uint8_t *comppred, int compstride,*/
                    tmp_buf, bwidth,  /*const uint8_t *interpred, int interstride,*/
                    intrapred, bwidth /*const uint8_t *intrapred,   int intrastride*/);


                model_rd_for_sb_with_curvfit(picture_control_set_ptr, context_ptr, context_ptr->blk_geom->bsize, bwidth, bheight,
                    src_buf, src_pic->stride_y, ii_pred_buf, bwidth,
                   0, 0, 0, 0, &rate_sum,  &dist_sum, NULL, NULL, NULL, NULL, NULL);

                rd = RDCOST(context_ptr->full_lambda, tmp_rate_mv + rate_sum + rmode, dist_sum);
                if (rd < best_interintra_rd) {
                    best_interintra_rd_wedge = rd;
                    best_interintra_mode = interintra_mode;

                    //CHKN added this as fix from lib-aom
                    best_interintra_rd = rd;
                }
            }

            candidate_ptr->interintra_mode      = best_interintra_mode;
            candidate_ptr->use_wedge_interintra =1 ;
        }
    }


}
#endif


#if COMP_MODE
COMPOUND_TYPE to_av1_compound_lut[] = {
    COMPOUND_AVERAGE,
    COMPOUND_DISTWTD,
    COMPOUND_DIFFWTD,
    COMPOUND_WEDGE
};

void determine_compound_mode(
    PictureControlSet            *picture_control_set_ptr,
    ModeDecisionContext          *context_ptr,
    ModeDecisionCandidate        *candidatePtr,
    MD_COMP_TYPE                 cur_type) {


    candidatePtr->interinter_comp.type = to_av1_compound_lut[cur_type];

    if (cur_type == MD_COMP_AVG) {

        candidatePtr->comp_group_idx = 0;
        candidatePtr->compound_idx = 1;
#if 0//COMP_AVG_DIST
        if(candidatePtr->merge_flag ==0)
            search_compound_avg_dist(
                picture_control_set_ptr,
                context_ptr,
                candidatePtr);
#endif
    }
    else if (cur_type == MD_COMP_DIST) {

        candidatePtr->comp_group_idx = 0;
        candidatePtr->compound_idx = 0;
    }
    else if (cur_type == MD_COMP_DIFF0) {

        //printf("NOTHERRRRRRRRRRRR");;
        candidatePtr->comp_group_idx = 1;
        candidatePtr->compound_idx = 1;
        candidatePtr->interinter_comp.mask_type = 55;
#if COMP_DIFF
        search_compound_diff_wedge(
            picture_control_set_ptr,
            context_ptr,
            candidatePtr
            );
#endif
    }
    //else if (cur_type == MD_COMP_DIFF1) {
    //    candidatePtr->comp_group_idx = 1;
    //    candidatePtr->compound_idx = 1;
    //    candidatePtr->interinter_comp.mask_type = 1;
    //}
    else if (cur_type == MD_COMP_WEDGE) {

        candidatePtr->comp_group_idx = 1;
        candidatePtr->compound_idx = 1;
        search_compound_diff_wedge(
            picture_control_set_ptr,
            context_ptr,
            candidatePtr
            );

        candidatePtr->interinter_comp.wedge_index = candidatePtr->interinter_comp.wedge_index;
        candidatePtr->interinter_comp.wedge_sign = candidatePtr->interinter_comp.wedge_sign;
    }
    else {
        printf("ERROR: not used comp type\n");
    }



}
#endif
void ChooseBestAv1MvPred(
    ModeDecisionContext            *context_ptr,
    struct MdRateEstimationContext      *md_rate_estimation_ptr,
    CodingUnit      *cu_ptr,
    MvReferenceFrame ref_frame,
    uint8_t              is_compound,
    PredictionMode    mode,              //NEW or NEW_NEW
    int16_t             mv0x,
    int16_t             mv0y,
    int16_t             mv1x,
    int16_t             mv1y,
    uint8_t             *bestDrlIndex,      // output
    IntMv             bestPredmv[2]      // output
)
{
    uint8_t              drli, maxDrlIndex;
    IntMv             nearestmv[2];
    IntMv             nearmv[2];
    IntMv             ref_mv[2];
    uint32_t             bestmvCost = 0xFFFFFFFF;
    MV                 mv;

    maxDrlIndex = GetMaxDrlIndex(cu_ptr->av1xd->ref_mv_count[ref_frame], mode);
    // maxDrlIndex = 1;

    for (drli = 0; drli < maxDrlIndex; drli++) {
        get_av1_mv_pred_drl(
            context_ptr,
            cu_ptr,
            ref_frame,
            is_compound,
            mode,
            drli,
            nearestmv,
            nearmv,
            ref_mv);

        //compute the rate for this drli Cand
        mv.row = mv0y;
        mv.col = mv0x;

        uint32_t mvRate = (uint32_t)av1_mv_bit_cost(
            &mv,
            &(ref_mv[0].as_mv),
            md_rate_estimation_ptr->nmv_vec_cost,
            md_rate_estimation_ptr->nmvcoststack,
            MV_COST_WEIGHT);

        if (is_compound) {
            mv.row = mv1y;
            mv.col = mv1x;

            mvRate += (uint32_t)av1_mv_bit_cost(
                &mv,
                &(ref_mv[1].as_mv),
                md_rate_estimation_ptr->nmv_vec_cost,
                md_rate_estimation_ptr->nmvcoststack,
                MV_COST_WEIGHT);
        }

        if (mvRate < bestmvCost) {
            bestmvCost = mvRate;
            *bestDrlIndex = drli;
            bestPredmv[0] = ref_mv[0];
            bestPredmv[1] = ref_mv[1];
        }
    }
}

/***************************************
* Mode Decision Candidate Ctor
***************************************/
EbErrorType mode_decision_candidate_buffer_ctor(
    ModeDecisionCandidateBuffer **buffer_dbl_ptr,
    uint64_t                       *fast_cost_ptr,
    uint64_t                       *full_cost_ptr,
    uint64_t                       *full_cost_skip_ptr,
    uint64_t                       *full_cost_merge_ptr)
{
    EbPictureBufferDescInitData pictureBufferDescInitData;
    EbPictureBufferDescInitData doubleWidthPictureBufferDescInitData;

    EbPictureBufferDescInitData ThirtyTwoWidthPictureBufferDescInitData;

    EbErrorType return_error = EB_ErrorNone;
    // Allocate Buffer
    ModeDecisionCandidateBuffer *bufferPtr;
    EB_MALLOC(ModeDecisionCandidateBuffer*, bufferPtr, sizeof(ModeDecisionCandidateBuffer), EB_N_PTR);
    *buffer_dbl_ptr = bufferPtr;

    // Init Picture Data
    pictureBufferDescInitData.max_width = MAX_SB_SIZE;
    pictureBufferDescInitData.max_height = MAX_SB_SIZE;
    pictureBufferDescInitData.bit_depth = EB_8BIT;
    pictureBufferDescInitData.color_format = EB_YUV420;
    pictureBufferDescInitData.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
    pictureBufferDescInitData.left_padding = 0;
    pictureBufferDescInitData.right_padding = 0;
    pictureBufferDescInitData.top_padding = 0;
    pictureBufferDescInitData.bot_padding = 0;
    pictureBufferDescInitData.split_mode = EB_FALSE;
    doubleWidthPictureBufferDescInitData.max_width = MAX_SB_SIZE;
    doubleWidthPictureBufferDescInitData.max_height = MAX_SB_SIZE;
    doubleWidthPictureBufferDescInitData.bit_depth = EB_16BIT;
    doubleWidthPictureBufferDescInitData.color_format = EB_YUV420;
    doubleWidthPictureBufferDescInitData.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
    doubleWidthPictureBufferDescInitData.left_padding = 0;
    doubleWidthPictureBufferDescInitData.right_padding = 0;
    doubleWidthPictureBufferDescInitData.top_padding = 0;
    doubleWidthPictureBufferDescInitData.bot_padding = 0;
    doubleWidthPictureBufferDescInitData.split_mode = EB_FALSE;

    ThirtyTwoWidthPictureBufferDescInitData.max_width = MAX_SB_SIZE;
    ThirtyTwoWidthPictureBufferDescInitData.max_height = MAX_SB_SIZE;
    ThirtyTwoWidthPictureBufferDescInitData.bit_depth = EB_32BIT;
    ThirtyTwoWidthPictureBufferDescInitData.color_format = EB_YUV420;
    ThirtyTwoWidthPictureBufferDescInitData.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
    ThirtyTwoWidthPictureBufferDescInitData.left_padding = 0;
    ThirtyTwoWidthPictureBufferDescInitData.right_padding = 0;
    ThirtyTwoWidthPictureBufferDescInitData.top_padding = 0;
    ThirtyTwoWidthPictureBufferDescInitData.bot_padding = 0;
    ThirtyTwoWidthPictureBufferDescInitData.split_mode = EB_FALSE;

    // Candidate Ptr
    bufferPtr->candidate_ptr = (ModeDecisionCandidate*)EB_NULL;

    // Video Buffers
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->prediction_ptr),
        (EbPtr)&pictureBufferDescInitData);

    // Video Buffers
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->prediction_ptr_temp),
        (EbPtr)&pictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->cfl_temp_prediction_ptr),
        (EbPtr)&pictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->residual_ptr),
        (EbPtr)&doubleWidthPictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->residual_quant_coeff_ptr),
        (EbPtr)&ThirtyTwoWidthPictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->recon_coeff_ptr),
        (EbPtr)&ThirtyTwoWidthPictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->recon_ptr),
        (EbPtr)&pictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    //Distortion
    bufferPtr->residual_luma_sad = 0;

    bufferPtr->full_lambda_rate = 0;

    // Costs
    bufferPtr->fast_cost_ptr = fast_cost_ptr;
    bufferPtr->full_cost_ptr = full_cost_ptr;
    bufferPtr->full_cost_skip_ptr = full_cost_skip_ptr;
    bufferPtr->full_cost_merge_ptr = full_cost_merge_ptr;
    return EB_ErrorNone;
}
#if TX_TYPE_LOSSLESS
EbErrorType mode_decision_scratch_candidate_buffer_ctor(
    ModeDecisionCandidateBuffer *bufferPtr)
{

    EbErrorType return_error = EB_ErrorNone;

    EbPictureBufferDescInitData pictureBufferDescInitData;
    EbPictureBufferDescInitData doubleWidthPictureBufferDescInitData;
    EbPictureBufferDescInitData ThirtyTwoWidthPictureBufferDescInitData;

    // Init Picture Data
    pictureBufferDescInitData.max_width = MAX_SB_SIZE;
    pictureBufferDescInitData.max_height = MAX_SB_SIZE;
    pictureBufferDescInitData.bit_depth = EB_8BIT;
    pictureBufferDescInitData.color_format = EB_YUV420;
    pictureBufferDescInitData.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
    pictureBufferDescInitData.left_padding = 0;
    pictureBufferDescInitData.right_padding = 0;
    pictureBufferDescInitData.top_padding = 0;
    pictureBufferDescInitData.bot_padding = 0;
    pictureBufferDescInitData.split_mode = EB_FALSE;
    doubleWidthPictureBufferDescInitData.max_width = MAX_SB_SIZE;
    doubleWidthPictureBufferDescInitData.max_height = MAX_SB_SIZE;
    doubleWidthPictureBufferDescInitData.bit_depth = EB_16BIT;
    doubleWidthPictureBufferDescInitData.color_format = EB_YUV420;
    doubleWidthPictureBufferDescInitData.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
    doubleWidthPictureBufferDescInitData.left_padding = 0;
    doubleWidthPictureBufferDescInitData.right_padding = 0;
    doubleWidthPictureBufferDescInitData.top_padding = 0;
    doubleWidthPictureBufferDescInitData.bot_padding = 0;
    doubleWidthPictureBufferDescInitData.split_mode = EB_FALSE;

    ThirtyTwoWidthPictureBufferDescInitData.max_width = MAX_SB_SIZE;
    ThirtyTwoWidthPictureBufferDescInitData.max_height = MAX_SB_SIZE;
    ThirtyTwoWidthPictureBufferDescInitData.bit_depth = EB_32BIT;
    ThirtyTwoWidthPictureBufferDescInitData.color_format = EB_YUV420;
    ThirtyTwoWidthPictureBufferDescInitData.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
    ThirtyTwoWidthPictureBufferDescInitData.left_padding = 0;
    ThirtyTwoWidthPictureBufferDescInitData.right_padding = 0;
    ThirtyTwoWidthPictureBufferDescInitData.top_padding = 0;
    ThirtyTwoWidthPictureBufferDescInitData.bot_padding = 0;
    ThirtyTwoWidthPictureBufferDescInitData.split_mode = EB_FALSE;

    // Video Buffers
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->prediction_ptr),
        (EbPtr)&pictureBufferDescInitData);

    // Video Buffers
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->prediction_ptr_temp),
        (EbPtr)&pictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->cfl_temp_prediction_ptr),
        (EbPtr)&pictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->residual_ptr),
        (EbPtr)&doubleWidthPictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->residual_quant_coeff_ptr),
        (EbPtr)&ThirtyTwoWidthPictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->recon_coeff_ptr),
        (EbPtr)&ThirtyTwoWidthPictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    return_error = eb_picture_buffer_desc_ctor(
        (EbPtr*)&(bufferPtr->recon_ptr),
        (EbPtr)&pictureBufferDescInitData);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;

    return EB_ErrorNone;
}
#endif
#if PRUNE_REF_FRAME_FRO_REC_PARTITION
uint8_t check_ref_beackout(
    PictureControlSet          *picture_control_set_ptr,
    struct ModeDecisionContext *context_ptr,
    uint8_t                     ref_frame_type,
    PART                        shape)
{
    uint8_t skip_candidate = 0;
    uint8_t ref_cnt = 0;
    uint8_t allowed_nsq_ref_th = (uint8_t)PRUNE_REC_TH;
    if (picture_control_set_ptr->parent_pcs_ptr->prune_ref_frame_for_rec_partitions) {
        if (shape != PART_N) {
            if (1/*is_inter && is_simple_translation*/) {
                uint8_t ref_idx;
                assert(ref_frame_type < 30);
                ref_cnt = 0;
                for (ref_idx = 0; ref_idx < allowed_nsq_ref_th; ref_idx++) {
                    if (ref_frame_type == context_ptr->ref_best_ref_sq_table[ref_idx]) {
                        ref_cnt ++;
                    }
                }
                skip_candidate = ref_cnt ? 0 : 1;
            }
        }
    }
    return skip_candidate;
}
#endif
// Function Declarations
void RoundMv(
    ModeDecisionCandidate    *candidateArray,
    uint32_t                   canTotalCnt)
{
    candidateArray[canTotalCnt].motion_vector_xl0 = (candidateArray[canTotalCnt].motion_vector_xl0 + 2)&~0x03;
    candidateArray[canTotalCnt].motion_vector_yl0 = (candidateArray[canTotalCnt].motion_vector_yl0 + 2)&~0x03;

    candidateArray[canTotalCnt].motion_vector_xl1 = (candidateArray[canTotalCnt].motion_vector_xl1 + 2)&~0x03;
    candidateArray[canTotalCnt].motion_vector_yl1 = (candidateArray[canTotalCnt].motion_vector_yl1 + 2)&~0x03;

    return;
}

/***************************************
* return true if the MV candidate is already injected
***************************************/
#if MRP_DUPLICATION_FIX
EbBool mrp_is_already_injected_mv_l0(
    ModeDecisionContext *context_ptr,
    int16_t                mv_x,
    int16_t                mv_y,
    uint8_t                ref_type) {
    for (int inter_candidate_index = 0; inter_candidate_index < context_ptr->injected_mv_count_l0; inter_candidate_index++) {
        if (context_ptr->injected_mv_x_l0_array[inter_candidate_index] == mv_x &&
            context_ptr->injected_mv_y_l0_array[inter_candidate_index] == mv_y &&
            context_ptr->injected_ref_type_l0_array[inter_candidate_index] == ref_type) {
            return(EB_TRUE);
        }
    }

    return(EB_FALSE);
}

EbBool mrp_is_already_injected_mv_l1(
    ModeDecisionContext *context_ptr,
    int16_t                mv_x,
    int16_t                mv_y,
    uint8_t                ref_type) {
    for (int inter_candidate_index = 0; inter_candidate_index < context_ptr->injected_mv_count_l1; inter_candidate_index++) {
        if (context_ptr->injected_mv_x_l1_array[inter_candidate_index] == mv_x &&
            context_ptr->injected_mv_y_l1_array[inter_candidate_index] == mv_y &&
            context_ptr->injected_ref_type_l1_array[inter_candidate_index] == ref_type) {
            return(EB_TRUE);
        }
    }

    return(EB_FALSE);
}

EbBool mrp_is_already_injected_mv_bipred(
    ModeDecisionContext *context_ptr,
    int16_t                mv_x_l0,
    int16_t                mv_y_l0,
    int16_t                mv_x_l1,
    int16_t                mv_y_l1,
    uint8_t                ref_type) {
    for (int inter_candidate_index = 0; inter_candidate_index < context_ptr->injected_mv_count_bipred; inter_candidate_index++) {
        if (context_ptr->injected_mv_x_bipred_l0_array[inter_candidate_index] == mv_x_l0 &&
            context_ptr->injected_mv_y_bipred_l0_array[inter_candidate_index] == mv_y_l0 &&
            context_ptr->injected_mv_x_bipred_l1_array[inter_candidate_index] == mv_x_l1 &&
            context_ptr->injected_mv_y_bipred_l1_array[inter_candidate_index] == mv_y_l1 &&
            context_ptr->injected_ref_type_bipred_array[inter_candidate_index] == ref_type) {
            return(EB_TRUE);
        }
    }
    return(EB_FALSE);
}
#else
EbBool is_already_injected_mv_l0(
    ModeDecisionContext *context_ptr,
    int16_t                mv_x,
    int16_t                mv_y) {
    for (int inter_candidate_index = 0; inter_candidate_index < context_ptr->injected_mv_count_l0; inter_candidate_index++) {
        if (context_ptr->injected_mv_x_l0_array[inter_candidate_index] == mv_x &&
            context_ptr->injected_mv_y_l0_array[inter_candidate_index] == mv_y) {
            return(EB_TRUE);
        }
    }

    return(EB_FALSE);
}

EbBool is_already_injected_mv_l1(
    ModeDecisionContext *context_ptr,
    int16_t                mv_x,
    int16_t                mv_y) {
    for (int inter_candidate_index = 0; inter_candidate_index < context_ptr->injected_mv_count_l1; inter_candidate_index++) {
        if (context_ptr->injected_mv_x_l1_array[inter_candidate_index] == mv_x &&
            context_ptr->injected_mv_y_l1_array[inter_candidate_index] == mv_y) {
            return(EB_TRUE);
        }
    }

    return(EB_FALSE);
}

EbBool is_already_injected_mv_bipred(
    ModeDecisionContext *context_ptr,
    int16_t                mv_x_l0,
    int16_t                mv_y_l0,
    int16_t                mv_x_l1,
    int16_t                mv_y_l1) {
    for (int inter_candidate_index = 0; inter_candidate_index < context_ptr->injected_mv_count_bipred; inter_candidate_index++) {
        if (context_ptr->injected_mv_x_bipred_l0_array[inter_candidate_index] == mv_x_l0 &&
            context_ptr->injected_mv_y_bipred_l0_array[inter_candidate_index] == mv_y_l0 &&
            context_ptr->injected_mv_x_bipred_l1_array[inter_candidate_index] == mv_x_l1 &&
            context_ptr->injected_mv_y_bipred_l1_array[inter_candidate_index] == mv_y_l1) {
            return(EB_TRUE);
        }
    }
    return(EB_FALSE);
}
#endif

EbErrorType SetMvpClipMVs(
    ModeDecisionCandidate  *candidate_ptr,
    uint32_t                    cu_origin_x,
    uint32_t                    cu_origin_y,
    uint32_t                    pu_index,
    uint32_t                    tb_size,
    PictureControlSet      *picture_control_set_ptr)
{
    EbErrorType  return_error = EB_ErrorNone;

    uint32_t        picture_width = ((SequenceControlSet*)picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr)->seq_header.max_frame_width;
    uint32_t        picture_height = ((SequenceControlSet*)picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr)->seq_header.max_frame_height;

    candidate_ptr->motion_vector_pred_idx[REF_LIST_0] = 0;
    candidate_ptr->motion_vector_pred_x[REF_LIST_0] = 0;
    candidate_ptr->motion_vector_pred_y[REF_LIST_0] = 0;
    candidate_ptr->motion_vector_pred_idx[REF_LIST_1] = 0;
    candidate_ptr->motion_vector_pred_x[REF_LIST_1] = 0;
    candidate_ptr->motion_vector_pred_y[REF_LIST_1] = 0;

    switch (candidate_ptr->prediction_direction[pu_index]) {
    case UNI_PRED_LIST_0:
        // Clip the input MV
        clip_mv(
            cu_origin_x,
            cu_origin_y,
            &candidate_ptr->motion_vector_xl0,
            &candidate_ptr->motion_vector_yl0,
            picture_width,
            picture_height,
            tb_size);

        break;

    case UNI_PRED_LIST_1:

        // Clip the input MV
        clip_mv(
            cu_origin_x,
            cu_origin_y,
            &candidate_ptr->motion_vector_xl1,
            &candidate_ptr->motion_vector_yl1,
            picture_width,
            picture_height,
            tb_size);

        break;

    case BI_PRED:

        // Choose the MVP in list0
        // Clip the input MV
        clip_mv(
            cu_origin_x,
            cu_origin_y,
            &candidate_ptr->motion_vector_xl0,
            &candidate_ptr->motion_vector_yl0,
            picture_width,
            picture_height,
            tb_size);

        // Choose the MVP in list1
        // Clip the input MV
        clip_mv(
            cu_origin_x,
            cu_origin_y,
            &candidate_ptr->motion_vector_xl1,
            &candidate_ptr->motion_vector_yl1,
            picture_width,
            picture_height,
            tb_size);
        break;

    default:
        break;
    }

    return return_error;
}

void LimitMvOverBound(
    int16_t *mvx,
    int16_t *mvy,
    ModeDecisionContext     *ctxtPtr,
    const SequenceControlSet      *sCSet)
{
    int32_t mvxF, mvyF;

    //L0
    mvxF = (*mvx) >> 2;
    mvyF = (*mvy) >> 2;

    if ((int32_t)ctxtPtr->cu_origin_x + mvxF + (int32_t)ctxtPtr->blk_geom->bwidth > (int32_t)sCSet->seq_header.max_frame_width)
        *mvx = (int16_t)(sCSet->seq_header.max_frame_width - ctxtPtr->blk_geom->bwidth - ctxtPtr->cu_origin_x);
    if ((int32_t)ctxtPtr->cu_origin_y + mvyF + (int32_t)ctxtPtr->blk_geom->bheight > (int32_t)sCSet->seq_header.max_frame_height)
        *mvy = (int16_t)(sCSet->seq_header.max_frame_height - ctxtPtr->blk_geom->bheight - ctxtPtr->cu_origin_y);
    if ((int32_t)ctxtPtr->cu_origin_x + mvxF < 0)
        *mvx = -(int16_t)ctxtPtr->cu_origin_x;
    if ((int32_t)ctxtPtr->cu_origin_y + mvyF < 0)
        *mvy = -(int16_t)ctxtPtr->cu_origin_y;
}
#if !MDLEVELS
void sort_fast_loop_candidates(
    struct ModeDecisionContext   *context_ptr,
    uint32_t                        buffer_total_count,
    ModeDecisionCandidateBuffer **buffer_ptr_array,
#if DECOUPLED_FAST_LOOP || MD_CLASS
    uint32_t                        *best_candidate_index_array,
    uint32_t                        *sorted_candidate_index_array,
#else
    uint8_t                        *best_candidate_index_array,
    uint8_t                        *sorted_candidate_index_array,
#endif
    uint64_t                       *ref_fast_cost) {
    uint32_t fullReconCandidateCount = context_ptr->full_recon_search_count;

    //  move the scratch candidates (MAX_CU_COST) to the last spots (if any)
    uint32_t best_candidate_start_index = 0;
    uint32_t best_candidate_end_index = buffer_total_count - 1;
#if DECOUPLED_FAST_LOOP || MD_CLASS
    for (uint32_t full_buffer_index = 0; full_buffer_index < buffer_total_count; full_buffer_index++) {
#else
    for (uint8_t full_buffer_index = 0; full_buffer_index < buffer_total_count; full_buffer_index++) {
#endif
        if (*(buffer_ptr_array[full_buffer_index]->fast_cost_ptr) == MAX_CU_COST)
            best_candidate_index_array[best_candidate_end_index--] = full_buffer_index;
        else
            best_candidate_index_array[best_candidate_start_index++] = full_buffer_index;
    }

    // fl escape: inter then intra
    uint32_t i, j, index;
#if DECOUPLED_FAST_LOOP
    for (i = 0; i < fullReconCandidateCount; ++i)
    best_candidate_index_array[i] = i;
    for (i = 0; i < fullReconCandidateCount - 1; ++i) {
        for (j = i + 1; j < fullReconCandidateCount; ++j) {
            if (*(buffer_ptr_array[best_candidate_index_array[j]]->fast_cost_ptr) < *(buffer_ptr_array[best_candidate_index_array[i]]->fast_cost_ptr)) {
                index = best_candidate_index_array[i];
                best_candidate_index_array[i] = (uint32_t)best_candidate_index_array[j];
                best_candidate_index_array[j] = (uint32_t)index;
            }
        }
    }
#else
    for (i = 0; i < fullReconCandidateCount - 1; ++i) {
        for (j = i + 1; j < fullReconCandidateCount; ++j) {
            if ((buffer_ptr_array[best_candidate_index_array[i]]->candidate_ptr->type == INTRA_MODE) &&
                (buffer_ptr_array[best_candidate_index_array[j]]->candidate_ptr->type == INTER_MODE)) {
                index = best_candidate_index_array[i];
#if DECOUPLED_FAST_LOOP || MD_CLASS
                best_candidate_index_array[i] = (uint32_t)best_candidate_index_array[j];
                best_candidate_index_array[j] = (uint32_t)index;
#else
                best_candidate_index_array[i] = (uint8_t)best_candidate_index_array[j];
                best_candidate_index_array[j] = (uint8_t)index;
#endif
            }
        }
    }
#endif
#if M9_FULL_LOOP_ESCAPE
    // fl escape level 2: inter then intra
    for (i = 0; i < fullReconCandidateCount; ++i)
        sorted_candidate_index_array[i] = best_candidate_index_array[i];

    for (i = 0; i < fullReconCandidateCount - 1; ++i) {
        for (j = i + 1; j < fullReconCandidateCount; ++j) {
            if (*(buffer_ptr_array[sorted_candidate_index_array[j]]->fast_cost_ptr) < *(buffer_ptr_array[sorted_candidate_index_array[i]]->fast_cost_ptr)) {
                index = sorted_candidate_index_array[i];
#if DECOUPLED_FAST_LOOP || MD_CLASS
                sorted_candidate_index_array[i] = (uint32_t)sorted_candidate_index_array[j];
                sorted_candidate_index_array[j] = (uint32_t)index;
#else
                sorted_candidate_index_array[i] = (uint8_t)sorted_candidate_index_array[j];
                sorted_candidate_index_array[j] = (uint8_t)index;
#endif
            }
        }
    }
    // tx search
    *ref_fast_cost = *(buffer_ptr_array[sorted_candidate_index_array[0]]->fast_cost_ptr);
#else
    // tx search
    for (i = 0; i < fullReconCandidateCount; i++) {
        if (*(buffer_ptr_array[i]->fast_cost_ptr) < *ref_fast_cost)
            *ref_fast_cost = *(buffer_ptr_array[i]->fast_cost_ptr);
    }
    for (i = 0; i < MAX_NFL; ++i)
        sorted_candidate_index_array[i] = best_candidate_index_array[i];
    for (i = 0; i < fullReconCandidateCount - 1; ++i) {
        for (j = i + 1; j < fullReconCandidateCount; ++j) {
            if (*(buffer_ptr_array[j]->fast_cost_ptr) < *(buffer_ptr_array[i]->fast_cost_ptr)) {
                index = sorted_candidate_index_array[i];
                sorted_candidate_index_array[i] = (uint8_t)sorted_candidate_index_array[j];
                sorted_candidate_index_array[j] = (uint8_t)index;
            }
        }
    }
#endif
}
#endif
#define BIPRED_3x3_REFINEMENT_POSITIONS 8

int8_t ALLOW_REFINEMENT_FLAG[BIPRED_3x3_REFINEMENT_POSITIONS] = {  1, 0, 1, 0, 1,  0,  1, 0 };
int8_t BIPRED_3x3_X_POS[BIPRED_3x3_REFINEMENT_POSITIONS] = { -1, -1, 0, 1, 1, 1, 0, -1 };
int8_t BIPRED_3x3_Y_POS[BIPRED_3x3_REFINEMENT_POSITIONS] = { 0, 1, 1, 1, 0, -1, -1, -1 };

void Unipred3x3CandidatesInjection(
#if MEMORY_FOOTPRINT_OPT_ME_MV
    const SequenceControlSet  *sequence_control_set_ptr,
#endif
    PictureControlSet         *picture_control_set_ptr,
    ModeDecisionContext       *context_ptr,
    LargestCodingUnit         *sb_ptr,
    uint32_t                   me_sb_addr,
    SsMeContext               *inloop_me_context,
    EbBool                     use_close_loop_me,
    uint32_t                   close_loop_me_index,
#if !MEMORY_FOOTPRINT_OPT_ME_MV
    const uint32_t             me2Nx2NTableOffset,
#endif
    uint32_t                  *candidateTotalCnt){
    UNUSED(sb_ptr);
    uint32_t                   bipredIndex;
    uint32_t                   canTotalCnt = (*candidateTotalCnt);
#if MD_INJECTION
#if MEMORY_FOOTPRINT_OPT_ME_MV
    const MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr];
    uint8_t total_me_cnt = me_results->total_me_candidate_index[context_ptr->me_block_offset];
    const MeCandidate *me_block_results = me_results->me_candidate[context_ptr->me_block_offset];
#else
    const MeLcuResults *me_results      = picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr];
    uint8_t total_me_cnt                = me_results->total_me_candidate_index[me2Nx2NTableOffset];
    const MeCandidate *me_block_results = me_results->me_candidate[me2Nx2NTableOffset];
#endif
#else
    MeCuResults * mePuResult = &picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr][me2Nx2NTableOffset];
#endif
    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
    EbBool isCompoundEnabled = (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
    IntMv  bestPredmv[2] = { {0}, {0} };

    // (8 Best_L0 neighbors)
#if MD_INJECTION
    //const MeLcuResults_t *meResults = pictureControlSetPtr->ParentPcsPtr->meResultsPtr[lcuAddr];
#if APPLY_3X3_FOR_BEST_ME
    total_me_cnt = MIN(total_me_cnt, BEST_CANDIDATE_COUNT);
#endif
    for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index)
    {
        const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
        const uint8_t inter_direction = me_block_results_ptr->direction;
        const uint8_t list0_ref_index = me_block_results_ptr->ref_idx_l0;

        if (inter_direction == 0) {
#endif
    for (bipredIndex = 0; bipredIndex < BIPRED_3x3_REFINEMENT_POSITIONS; ++bipredIndex)
    {
        /**************
        NEWMV L0
        ************* */
        if (context_ptr->unipred3x3_injection >= 2){
            if (ALLOW_REFINEMENT_FLAG[bipredIndex] == 0)
                continue;
        }
#if MD_INJECTION
#if MEMORY_FOOTPRINT_OPT_ME_MV
        int16_t to_inject_mv_x = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1 : (me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].x_mv + BIPRED_3x3_X_POS[bipredIndex]) << 1;
        int16_t to_inject_mv_y = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1 : (me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].y_mv + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#else
        int16_t to_inject_mv_x = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1 : (me_block_results_ptr->x_mv_l0 + BIPRED_3x3_X_POS[bipredIndex]) << 1;
        int16_t to_inject_mv_y = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1 : (me_block_results_ptr->y_mv_l0 + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#endif
#else
        int16_t to_inject_mv_x = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1 : (mePuResult->x_mv_l0 + BIPRED_3x3_X_POS[bipredIndex]) << 1;
        int16_t to_inject_mv_y = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1 : (mePuResult->y_mv_l0 + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#endif
#if MRP_DUPLICATION_FIX
        uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
#if PRUNE_REF_FRAME_FRO_REC_PARTITION
        uint8_t skip_cand = check_ref_beackout(
            picture_control_set_ptr,
            context_ptr,
            to_inject_ref_type,
            context_ptr->blk_geom->shape);

        if (!skip_cand && (context_ptr->injected_mv_count_l0 == 0 || mrp_is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE)) {
#else
        if (context_ptr->injected_mv_count_l0 == 0 || mrp_is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE) {
#endif
#else
        if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
#endif
#if II_SEARCH // 3x3  L0
             MvReferenceFrame rf[2];
             rf[0] = to_inject_ref_type;
             rf[1] = -1;
            uint8_t tot_ii_types =    1;//svt_is_interintra_allowed(picture_control_set_ptr->parent_pcs_ptr->enable_inter_intra,context_ptr->blk_geom->bsize, NEWMV, rf) ? II_COUNT : 1;
            uint8_t ii_type;
            for (ii_type = 0; ii_type < tot_ii_types; ii_type++)
            {
#endif
            candidateArray[canTotalCnt].type = INTER_MODE;
            candidateArray[canTotalCnt].distortion_ready = 0;
            candidateArray[canTotalCnt].use_intrabc = 0;
            candidateArray[canTotalCnt].merge_flag = EB_FALSE;
            candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)0;
            candidateArray[canTotalCnt].inter_mode = NEWMV;
            candidateArray[canTotalCnt].pred_mode = NEWMV;
            candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;

            candidateArray[canTotalCnt].is_compound = 0;
            candidateArray[canTotalCnt].is_new_mv = 1;
            candidateArray[canTotalCnt].is_zero_mv = 0;

            candidateArray[canTotalCnt].drl_index = 0;

            // Set the MV to ME result
            candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x;
            candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y;

            // will be needed later by the rate estimation
            candidateArray[canTotalCnt].ref_mv_index = 0;
            candidateArray[canTotalCnt].pred_mv_weight = 0;
#if MRP_LIST_REF_IDX_TYPE_LT
            candidateArray[canTotalCnt].ref_frame_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
            candidateArray[canTotalCnt].ref_frame_index_l0 = list0_ref_index;
            candidateArray[canTotalCnt].ref_frame_index_l1 = -1;
#else
            candidateArray[canTotalCnt].ref_frame_type = LAST_FRAME;
#endif

#if ATB_TX_TYPE_SUPPORT_PER_TU
            candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
            candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

            ChooseBestAv1MvPred(
                context_ptr,
                candidateArray[canTotalCnt].md_rate_estimation_ptr,
                context_ptr->cu_ptr,
                candidateArray[canTotalCnt].ref_frame_type,
                candidateArray[canTotalCnt].is_compound,
                candidateArray[canTotalCnt].pred_mode,
                candidateArray[canTotalCnt].motion_vector_xl0,
                candidateArray[canTotalCnt].motion_vector_yl0,
                0, 0,
                &candidateArray[canTotalCnt].drl_index,
                bestPredmv);

            candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
            candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
#if II_SEARCH
                candidateArray[canTotalCnt].is_interintra_used = ii_type == 0 ? 0 : 1;
                if (ii_type == 1) {
                    inter_intra_search(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canTotalCnt]);

                    candidateArray[canTotalCnt].use_wedge_interintra = 1;
                    candidateArray[canTotalCnt].ii_wedge_sign = 0;

                }
                if (ii_type == 2)// smooth
                {
                    candidateArray[canTotalCnt].interintra_mode = candidateArray[canTotalCnt-1].interintra_mode;
                    candidateArray[canTotalCnt].use_wedge_interintra = 0;
                }
#endif
#if CHECK_CAND
            INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
            ++canTotalCnt;
#endif
#if II_SEARCH
            }
#endif
            context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
            context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
#if MRP_DUPLICATION_FIX
            context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] = to_inject_ref_type;
#endif
            ++context_ptr->injected_mv_count_l0;
        }
#if MD_INJECTION
           }
        }
#endif
    }

    // (8 Best_L1 neighbors)
#if MD_INJECTION
//const MeLcuResults_t *meResults = pictureControlSetPtr->ParentPcsPtr->meResultsPtr[lcuAddr];
#if APPLY_3X3_FOR_BEST_ME
    total_me_cnt = MIN(total_me_cnt, BEST_CANDIDATE_COUNT);
#endif
    for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index)
    {
        const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
        const uint8_t inter_direction = me_block_results_ptr->direction;
        const uint8_t list1_ref_index = me_block_results_ptr->ref_idx_l1;
        if (inter_direction == 1) {
#endif
    for (bipredIndex = 0; bipredIndex < BIPRED_3x3_REFINEMENT_POSITIONS; ++bipredIndex)
    {
        if (isCompoundEnabled) {
            /**************
            NEWMV L1
            ************* */
            if (context_ptr->unipred3x3_injection >= 2) {
                if (ALLOW_REFINEMENT_FLAG[bipredIndex] == 0)
                    continue;
            }
#if MD_INJECTION
#if MEMORY_FOOTPRINT_OPT_ME_MV
#if FROM_7_TO_4_MV
            int16_t to_inject_mv_x = use_close_loop_me ? (inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1 : (me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2) + list1_ref_index].x_mv + BIPRED_3x3_X_POS[bipredIndex]) << 1;
            int16_t to_inject_mv_y = use_close_loop_me ? (inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1 : (me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2) + list1_ref_index].y_mv + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#else
            int16_t to_inject_mv_x = use_close_loop_me ? (inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1 : (me_results->me_mv_array[me2Nx2NTableOffset][4 + list1_ref_index].x_mv + BIPRED_3x3_X_POS[bipredIndex]) << 1;
            int16_t to_inject_mv_y = use_close_loop_me ? (inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1 : (me_results->me_mv_array[me2Nx2NTableOffset][4 + list1_ref_index].y_mv + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#endif
#else
            int16_t to_inject_mv_x = use_close_loop_me ? (inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1 : (me_block_results_ptr->x_mv_l1 + BIPRED_3x3_X_POS[bipredIndex]) << 1;
            int16_t to_inject_mv_y = use_close_loop_me ? (inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1 : (me_block_results_ptr->y_mv_l1 + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#endif
#else
            int16_t to_inject_mv_x = use_close_loop_me ? (inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1 : (mePuResult->x_mv_l1 + BIPRED_3x3_X_POS[bipredIndex]) << 1;
            int16_t to_inject_mv_y = use_close_loop_me ? (inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1 : (mePuResult->y_mv_l1 + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#endif
#if MRP_DUPLICATION_FIX
            uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#if PRUNE_REF_FRAME_FRO_REC_PARTITION
            uint8_t skip_cand = check_ref_beackout(
                picture_control_set_ptr,
                context_ptr,
                to_inject_ref_type,
                context_ptr->blk_geom->shape);

            if (!skip_cand && (context_ptr->injected_mv_count_l1 == 0 || mrp_is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE)) {
#else
            if (context_ptr->injected_mv_count_l1 == 0 || mrp_is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE) {
#endif
#else
            if (context_ptr->injected_mv_count_l1 == 0 || is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
#endif
#if II_SEARCH // 3x3  L1
             MvReferenceFrame rf[2];
             rf[0] = to_inject_ref_type;
             rf[1] = -1;
            uint8_t tot_ii_types =    1;//svt_is_interintra_allowed(picture_control_set_ptr->parent_pcs_ptr->enable_inter_intra,context_ptr->blk_geom->bsize, NEWMV, rf) ? II_COUNT : 1;
            uint8_t ii_type;
            for (ii_type = 0; ii_type < tot_ii_types; ii_type++)
            {
#endif
                candidateArray[canTotalCnt].type = INTER_MODE;
                candidateArray[canTotalCnt].distortion_ready = 0;
                candidateArray[canTotalCnt].use_intrabc = 0;
                candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)1;
                candidateArray[canTotalCnt].inter_mode = NEWMV;
                candidateArray[canTotalCnt].pred_mode = NEWMV;
                candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;

                candidateArray[canTotalCnt].is_compound = 0;
                candidateArray[canTotalCnt].is_new_mv = 1;
                candidateArray[canTotalCnt].is_zero_mv = 0;

                candidateArray[canTotalCnt].drl_index = 0;

                // Set the MV to ME result
                candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x;
                candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y;
                // will be needed later by the rate estimation
                candidateArray[canTotalCnt].ref_mv_index = 0;
                candidateArray[canTotalCnt].pred_mv_weight = 0;
#if MRP_LIST_REF_IDX_TYPE_LT
                candidateArray[canTotalCnt].ref_frame_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
                candidateArray[canTotalCnt].ref_frame_index_l0 = -1;
                candidateArray[canTotalCnt].ref_frame_index_l1 = list1_ref_index;
#else
                candidateArray[canTotalCnt].ref_frame_type = BWDREF_FRAME;
#endif
#if ATB_TX_TYPE_SUPPORT_PER_TU
                candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif
                ChooseBestAv1MvPred(
                    context_ptr,
                    candidateArray[canTotalCnt].md_rate_estimation_ptr,
                    context_ptr->cu_ptr,
                    candidateArray[canTotalCnt].ref_frame_type,
                    candidateArray[canTotalCnt].is_compound,
                    candidateArray[canTotalCnt].pred_mode,
                    candidateArray[canTotalCnt].motion_vector_xl1,
                    candidateArray[canTotalCnt].motion_vector_yl1,
                    0, 0,
                    &candidateArray[canTotalCnt].drl_index,
                    bestPredmv);

                candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[0].as_mv.col;
                candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[0].as_mv.row;
#if II_SEARCH
                candidateArray[canTotalCnt].is_interintra_used = ii_type == 0 ? 0 : 1;
                if (ii_type == 1) {
                    inter_intra_search(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canTotalCnt]);

                    candidateArray[canTotalCnt].use_wedge_interintra = 1;
                    candidateArray[canTotalCnt].ii_wedge_sign = 0;

                }
                if (ii_type == 2)// smooth
                {
                    candidateArray[canTotalCnt].interintra_mode = candidateArray[canTotalCnt-1].interintra_mode;
                    candidateArray[canTotalCnt].use_wedge_interintra = 0;
                }
#endif
#if CHECK_CAND
                INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
                ++canTotalCnt;
#endif
#if II_SEARCH
            }
#endif
                context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_x;
                context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_y;
#if MRP_DUPLICATION_FIX
                context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] = to_inject_ref_type;
#endif
                ++context_ptr->injected_mv_count_l1;
            }
        }
#if MD_INJECTION
    }
        }
#endif
    }

    // update the total number of candidates injected
    (*candidateTotalCnt) = canTotalCnt;

    return;
}

void Bipred3x3CandidatesInjection(
#if MEMORY_FOOTPRINT_OPT_ME_MV
    const SequenceControlSet *sequence_control_set_ptr,
#endif
    PictureControlSet        *picture_control_set_ptr,
    ModeDecisionContext      *context_ptr,
    LargestCodingUnit        *sb_ptr,
    uint32_t                  me_sb_addr,
    SsMeContext              *inloop_me_context,
    EbBool                    use_close_loop_me,
    uint32_t                  close_loop_me_index,
#if !MEMORY_FOOTPRINT_OPT_ME_MV
    const uint32_t            me2Nx2NTableOffset,
#endif
    uint32_t                 *candidateTotalCnt){
    UNUSED(sb_ptr);
    uint32_t                   bipredIndex;
    uint32_t                   canTotalCnt = (*candidateTotalCnt);
#if MD_INJECTION
#if MEMORY_FOOTPRINT_OPT_ME_MV
    const MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr];
    uint8_t total_me_cnt = me_results->total_me_candidate_index[context_ptr->me_block_offset];
    const MeCandidate *me_block_results = me_results->me_candidate[context_ptr->me_block_offset];
#else
    const MeLcuResults *me_results      = picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr];
    uint8_t total_me_cnt                = me_results->total_me_candidate_index[me2Nx2NTableOffset];
    const MeCandidate *me_block_results = me_results->me_candidate[me2Nx2NTableOffset];
#endif
#else
    MeCuResults * mePuResult = &picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr][me2Nx2NTableOffset];
#endif
    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
    EbBool isCompoundEnabled = (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
    IntMv  bestPredmv[2] = { {0}, {0} };
#if COMP_MODE
    BlockSize bsize = context_ptr->blk_geom->bsize;                       // bloc size
    MD_COMP_TYPE compound_types_to_try = picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;
    MD_COMP_TYPE cur_type; //BIP 3x3 MiSize >= BLOCK_8X8 && MiSize <= BLOCK_32X32)
#if COMP_OPT
    MD_COMP_TYPE tot_comp_types = picture_control_set_ptr->parent_pcs_ptr->compound_mode == 1 ?  MD_COMP_AVG :
        (bsize >= BLOCK_8X8 && bsize <= BLOCK_32X32) ? compound_types_to_try :
        (compound_types_to_try == MD_COMP_WEDGE) ? MD_COMP_DIFF0 :
        picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;


#if N0_COMP
    tot_comp_types = picture_control_set_ptr->enc_mode == ENC_M0 ? MD_COMP_AVG : tot_comp_types;
#endif

#else
    MD_COMP_TYPE tot_comp_types = (bsize >= BLOCK_8X8 && bsize<= BLOCK_32X32 ) ? compound_types_to_try :
                                  (compound_types_to_try == MD_COMP_WEDGE )? MD_COMP_DIFF0 :
                                   picture_control_set_ptr->parent_pcs_ptr->sequence_control_set_ptr->compound_types_to_try;//MD_COMP_DIST;// MD_COMP_AVG;//
#endif

#endif

    if (isCompoundEnabled) {
        /**************
       NEW_NEWMV
       ************* */
#if MD_INJECTION
       //const MeLcuResults_t *meResults = pictureControlSetPtr->ParentPcsPtr->meResultsPtr[lcuAddr];
#if APPLY_3X3_FOR_BEST_ME
        total_me_cnt = MIN(total_me_cnt, BEST_CANDIDATE_COUNT);
#endif
        for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index)
        {
            const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
            const uint8_t inter_direction = me_block_results_ptr->direction;
            const uint8_t list0_ref_index = me_block_results_ptr->ref_idx_l0;
            const uint8_t list1_ref_index = me_block_results_ptr->ref_idx_l1;

            if (inter_direction == 2) {
#endif
       // (Best_L0, 8 Best_L1 neighbors)
                for (bipredIndex = 0; bipredIndex < BIPRED_3x3_REFINEMENT_POSITIONS; ++bipredIndex)
                {
        if (context_ptr->bipred3x3_injection >= 2){
                        if (ALLOW_REFINEMENT_FLAG[bipredIndex] == 0)
                            continue;
                    }
#if MD_INJECTION
#if MEMORY_FOOTPRINT_OPT_ME_MV
                    int16_t to_inject_mv_x_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].x_mv << 1;
                    int16_t to_inject_mv_y_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].y_mv << 1;
#if FROM_7_TO_4_MV
                    int16_t to_inject_mv_x_l1 = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1) : (me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].x_mv + BIPRED_3x3_X_POS[bipredIndex]) << 1;
                    int16_t to_inject_mv_y_l1 = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1) : (me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].y_mv + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#else
                    int16_t to_inject_mv_x_l1 = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1) : (me_results->me_mv_array[me2Nx2NTableOffset][(me_block_results_ptr->ref1_list << 2) + list1_ref_index].x_mv + BIPRED_3x3_X_POS[bipredIndex]) << 1;
                    int16_t to_inject_mv_y_l1 = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1) : (me_results->me_mv_array[me2Nx2NTableOffset][(me_block_results_ptr->ref1_list << 2) + list1_ref_index].y_mv + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#endif
#else
                    int16_t to_inject_mv_x_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_block_results_ptr->x_mv_l0 << 1;
                    int16_t to_inject_mv_y_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_block_results_ptr->y_mv_l0 << 1;
                    int16_t to_inject_mv_x_l1 = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1) : (me_block_results_ptr->x_mv_l1 + BIPRED_3x3_X_POS[bipredIndex]) << 1;
                    int16_t to_inject_mv_y_l1 = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1) : (me_block_results_ptr->y_mv_l1 + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#endif
#else
                    int16_t to_inject_mv_x_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : mePuResult->x_mv_l0 << 1;
                    int16_t to_inject_mv_y_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : mePuResult->y_mv_l0 << 1;
                    int16_t to_inject_mv_x_l1 = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1) : (mePuResult->x_mv_l1 + BIPRED_3x3_X_POS[bipredIndex]) << 1;
                    int16_t to_inject_mv_y_l1 = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1) : (mePuResult->y_mv_l1 + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#endif

#if MRP_DUPLICATION_FIX
                    MvReferenceFrame rf[2];
#if MRP_MD_UNI_DIR_BIPRED // move it out side the loop
                    rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
                    rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
#else
                    rf[0] = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                    rf[1] = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#endif
                    uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
#if PRUNE_REF_FRAME_FRO_REC_PARTITION
                    uint8_t skip_cand = check_ref_beackout(
                        picture_control_set_ptr,
                        context_ptr,
                        to_inject_ref_type,
                        context_ptr->blk_geom->shape);

                    if (!skip_cand && (context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, to_inject_ref_type) == EB_FALSE)) {
#else
        if (context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, to_inject_ref_type) == EB_FALSE) {
#endif
#else
            if (context_ptr->injected_mv_count_bipred == 0 || is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1) == EB_FALSE) {
#endif
#if COMP_MODE




                context_ptr->variance_ready = 0 ;
                    for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types ; cur_type++)
                {
                // If two predictors are very similar, skip wedge compound mode search
                if (context_ptr->variance_ready)
                    if (context_ptr->prediction_mse < 8 || (!have_newmv_in_inter_mode(NEW_NEWMV) && context_ptr->prediction_mse  < 64))
                        continue;
            //if (mse < 8 || (!have_newmv_in_inter_mode(this_mode) && mse < 64)) {
            //  *comp_model_rd_cur = INT64_MAX;
            //  r
#endif
                candidateArray[canTotalCnt].type = INTER_MODE;
                candidateArray[canTotalCnt].distortion_ready = 0;
                candidateArray[canTotalCnt].use_intrabc = 0;
                candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                candidateArray[canTotalCnt].is_new_mv = 1;
                candidateArray[canTotalCnt].is_zero_mv = 0;

            candidateArray[canTotalCnt].drl_index = 0;

            // Set the MV to ME result
            candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x_l0;
            candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y_l0;
            candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x_l1;
            candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y_l1;
            // will be needed later by the rate estimation
            candidateArray[canTotalCnt].ref_mv_index = 0;
            candidateArray[canTotalCnt].pred_mv_weight = 0;

            candidateArray[canTotalCnt].inter_mode = NEW_NEWMV;
            candidateArray[canTotalCnt].pred_mode = NEW_NEWMV;
            candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
            candidateArray[canTotalCnt].is_compound = 1;
#if II_COMP
            candidateArray[canTotalCnt].is_interintra_used = 0;
#endif
            candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)2;
#if MRP_LIST_REF_IDX_TYPE_LT
            MvReferenceFrame rf[2];
#if MRP_MD_UNI_DIR_BIPRED
            rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
            rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
#else
            rf[0] = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
            rf[1] = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#endif
            candidateArray[canTotalCnt].ref_frame_type = av1_ref_frame_type(rf);
            candidateArray[canTotalCnt].ref_frame_index_l0 = list0_ref_index;
            candidateArray[canTotalCnt].ref_frame_index_l1 = list1_ref_index;
#else
            candidateArray[canTotalCnt].ref_frame_type = LAST_BWD_FRAME;
#endif
#if ATB_TX_TYPE_SUPPORT_PER_TU
            candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
            candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif
            ChooseBestAv1MvPred(
                context_ptr,
                candidateArray[canTotalCnt].md_rate_estimation_ptr,
                context_ptr->cu_ptr,
                candidateArray[canTotalCnt].ref_frame_type,
                candidateArray[canTotalCnt].is_compound,
                candidateArray[canTotalCnt].pred_mode,
                candidateArray[canTotalCnt].motion_vector_xl0,
                candidateArray[canTotalCnt].motion_vector_yl0,
                candidateArray[canTotalCnt].motion_vector_xl1,
                candidateArray[canTotalCnt].motion_vector_yl1,
                &candidateArray[canTotalCnt].drl_index,
                bestPredmv);

            candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
            candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
            candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[1].as_mv.col;
            candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[1].as_mv.row;
#if COMP_MODE
            //BIP 3x3
            determine_compound_mode(
                picture_control_set_ptr,
                context_ptr,
                &candidateArray[canTotalCnt],
                cur_type);
#endif
#if CHECK_CAND
            INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
            ++canTotalCnt;
#endif
            context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
            context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
            context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
            context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
#if MRP_DUPLICATION_FIX
            context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
#endif
            ++context_ptr->injected_mv_count_bipred;
#if COMP_MODE
                }
#endif
        }
        }

        // (8 Best_L0 neighbors, Best_L1) :
        for (bipredIndex = 0; bipredIndex < BIPRED_3x3_REFINEMENT_POSITIONS; ++bipredIndex)
        {
            if (context_ptr->bipred3x3_injection >= 2){
                if (ALLOW_REFINEMENT_FLAG[bipredIndex] == 0)
                    continue;
            }
#if MD_INJECTION
#if MEMORY_FOOTPRINT_OPT_ME_MV
            int16_t to_inject_mv_x_l0 = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1 : (me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].x_mv + BIPRED_3x3_X_POS[bipredIndex]) << 1;
            int16_t to_inject_mv_y_l0 = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1 : (me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].y_mv + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
#if FROM_7_TO_4_MV
            int16_t to_inject_mv_x_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].x_mv << 1;
            int16_t to_inject_mv_y_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].y_mv << 1;
#else
            int16_t to_inject_mv_x_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[me2Nx2NTableOffset][(me_block_results_ptr->ref1_list << 2) + list1_ref_index].x_mv << 1;
            int16_t to_inject_mv_y_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[me2Nx2NTableOffset][(me_block_results_ptr->ref1_list << 2) + list1_ref_index].y_mv << 1;
#endif
#else
            int16_t to_inject_mv_x_l0 = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1 : (me_block_results_ptr->x_mv_l0 + BIPRED_3x3_X_POS[bipredIndex]) << 1;
            int16_t to_inject_mv_y_l0 = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1 : (me_block_results_ptr->y_mv_l0 + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
            int16_t to_inject_mv_x_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_block_results_ptr->x_mv_l1 << 1;
            int16_t to_inject_mv_y_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_block_results_ptr->y_mv_l1 << 1;
#endif
#else
            int16_t to_inject_mv_x_l0 = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] + BIPRED_3x3_X_POS[bipredIndex]) << 1 : (mePuResult->x_mv_l0 + BIPRED_3x3_X_POS[bipredIndex]) << 1;
            int16_t to_inject_mv_y_l0 = use_close_loop_me ? (inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] + BIPRED_3x3_Y_POS[bipredIndex]) << 1 : (mePuResult->y_mv_l0 + BIPRED_3x3_Y_POS[bipredIndex]) << 1;
            int16_t to_inject_mv_x_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : mePuResult->x_mv_l1 << 1;
            int16_t to_inject_mv_y_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : mePuResult->y_mv_l1 << 1;
#endif

#if MRP_DUPLICATION_FIX
            MvReferenceFrame rf[2];
#if MRP_MD_UNI_DIR_BIPRED
            rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
            rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
#else
            rf[0] = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
            rf[1] = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#endif
            uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
#if PRUNE_REF_FRAME_FRO_REC_PARTITION
                    uint8_t skip_cand = check_ref_beackout(
                        picture_control_set_ptr,
                        context_ptr,
                        to_inject_ref_type,
                        context_ptr->blk_geom->shape);

            if (!skip_cand && (context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, to_inject_ref_type) == EB_FALSE)) {
#else
            if (context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, to_inject_ref_type) == EB_FALSE) {
#endif
#else
            if (context_ptr->injected_mv_count_bipred == 0 || is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1) == EB_FALSE) {
#endif
#if COMP_MODE

                context_ptr->variance_ready = 0 ;
                    for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types ; cur_type++)
                {
                // If two predictors are very similar, skip wedge compound mode search
                if (context_ptr->variance_ready)
                    if (context_ptr->prediction_mse < 8 || (!have_newmv_in_inter_mode(NEW_NEWMV) && context_ptr->prediction_mse  < 64))
                        continue;
#endif
                candidateArray[canTotalCnt].type = INTER_MODE;
                candidateArray[canTotalCnt].distortion_ready = 0;
                candidateArray[canTotalCnt].use_intrabc = 0;
                candidateArray[canTotalCnt].merge_flag = EB_FALSE;

                candidateArray[canTotalCnt].is_new_mv = 1;
                candidateArray[canTotalCnt].is_zero_mv = 0;

                candidateArray[canTotalCnt].drl_index = 0;

                // Set the MV to ME result
                candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x_l0;
                candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y_l0;
                candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x_l1;
                candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y_l1;
                // will be needed later by the rate estimation
                candidateArray[canTotalCnt].ref_mv_index = 0;
                candidateArray[canTotalCnt].pred_mv_weight = 0;

                candidateArray[canTotalCnt].inter_mode = NEW_NEWMV;
                candidateArray[canTotalCnt].pred_mode = NEW_NEWMV;
                candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                candidateArray[canTotalCnt].is_compound = 1;
#if II_COMP
                candidateArray[canTotalCnt].is_interintra_used = 0;
#endif
                candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)2;
#if MRP_LIST_REF_IDX_TYPE_LT
                MvReferenceFrame rf[2];
#if MRP_MD_UNI_DIR_BIPRED
                rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
                rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
#else
                rf[0] = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                rf[1] = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#endif
                candidateArray[canTotalCnt].ref_frame_type = av1_ref_frame_type(rf);
                candidateArray[canTotalCnt].ref_frame_index_l0 = list0_ref_index;
                candidateArray[canTotalCnt].ref_frame_index_l1 = list1_ref_index;
#else
                candidateArray[canTotalCnt].ref_frame_type = LAST_BWD_FRAME;
#endif
#if ATB_TX_TYPE_SUPPORT_PER_TU
                candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif
                ChooseBestAv1MvPred(
                    context_ptr,
                    candidateArray[canTotalCnt].md_rate_estimation_ptr,
                    context_ptr->cu_ptr,
                    candidateArray[canTotalCnt].ref_frame_type,
                    candidateArray[canTotalCnt].is_compound,
                    candidateArray[canTotalCnt].pred_mode,
                    candidateArray[canTotalCnt].motion_vector_xl0,
                    candidateArray[canTotalCnt].motion_vector_yl0,
                    candidateArray[canTotalCnt].motion_vector_xl1,
                    candidateArray[canTotalCnt].motion_vector_yl1,
                    &candidateArray[canTotalCnt].drl_index,
                    bestPredmv);

                candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
                candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[1].as_mv.col;
                candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[1].as_mv.row;
#if COMP_MODE
                    //BIP 3x3
                    determine_compound_mode(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canTotalCnt],
                        cur_type);
#endif
#if CHECK_CAND
                INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
                ++canTotalCnt;
#endif
                context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
#if MRP_DUPLICATION_FIX
                context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
#endif
                ++context_ptr->injected_mv_count_bipred;
#if COMP_MODE
                }
#endif
            }
        }
#if MD_INJECTION
            }
        }
#endif
     }

    // update the total number of candidates injected
    (*candidateTotalCnt) = canTotalCnt;

    return;
}
#if 0//EIGTH_PEL_MV
void eighth_pel_unipred_refinement(
    const SequenceControlSet  *sequence_control_set_ptr,
    PictureControlSet         *picture_control_set_ptr,
    ModeDecisionContext       *context_ptr,
    uint32_t                   me_sb_addr,
    SsMeContext               *inloop_me_context,
    EbBool                     use_close_loop_me,
    uint32_t                   close_loop_me_index,
    uint32_t                  *candidateTotalCnt) {
    uint32_t                   bipredIndex;
    uint32_t                   canTotalCnt = (*candidateTotalCnt);
    const MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr];
    uint8_t total_me_cnt = me_results->total_me_candidate_index[context_ptr->me_block_offset];
    const MeCandidate *me_block_results = me_results->me_candidate[context_ptr->me_block_offset];
    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
    EbBool isCompoundEnabled = (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
    IntMv  bestPredmv[2] = { {0}, {0} };

    // (8 Best_L0 neighbors)
    for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index) {
        const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
        const uint8_t inter_direction = me_block_results_ptr->direction;
        const uint8_t list0_ref_index = me_block_results_ptr->ref_idx_l0;
        if (inter_direction == 0) {
            for (bipredIndex = 0; bipredIndex < BIPRED_3x3_REFINEMENT_POSITIONS; ++bipredIndex) {
                /**************
                NEWMV L0
                ************* */
                if (context_ptr->unipred3x3_injection >= 2) {
                    if (ALLOW_REFINEMENT_FLAG[bipredIndex] == 0)
                        continue;
                }

                int16_t to_inject_mv_x = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] << 1) + BIPRED_3x3_X_POS[bipredIndex])  : ((me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].x_mv << 1) + BIPRED_3x3_X_POS[bipredIndex]);
                int16_t to_inject_mv_y = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] << 1) + BIPRED_3x3_Y_POS[bipredIndex])  : ((me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].y_mv << 1) + BIPRED_3x3_Y_POS[bipredIndex]);
                uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);

                if (context_ptr->injected_mv_count_l0 == 0 || mrp_is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE) {
                    candidateArray[canTotalCnt].type = INTER_MODE;
                    candidateArray[canTotalCnt].distortion_ready = 0;
                    candidateArray[canTotalCnt].use_intrabc = 0;
                    candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                    candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)0;
                    candidateArray[canTotalCnt].inter_mode = NEWMV;
                    candidateArray[canTotalCnt].pred_mode = NEWMV;
                    candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                    candidateArray[canTotalCnt].is_compound = 0;
                    candidateArray[canTotalCnt].is_new_mv = 1;
                    candidateArray[canTotalCnt].is_zero_mv = 0;
                    candidateArray[canTotalCnt].drl_index = 0;
                    // Set the MV to ME result
                    candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x;
                    candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y;
                    // will be needed later by the rate estimation
                    candidateArray[canTotalCnt].ref_mv_index = 0;
                    candidateArray[canTotalCnt].pred_mv_weight = 0;
                    candidateArray[canTotalCnt].ref_frame_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                    candidateArray[canTotalCnt].ref_frame_index_l0 = list0_ref_index;
                    candidateArray[canTotalCnt].ref_frame_index_l1 = -1;
                    candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                    candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

                    ChooseBestAv1MvPred(
                        context_ptr,
                        candidateArray[canTotalCnt].md_rate_estimation_ptr,
                        context_ptr->cu_ptr,
                        candidateArray[canTotalCnt].ref_frame_type,
                        candidateArray[canTotalCnt].is_compound,
                        candidateArray[canTotalCnt].pred_mode,
                        candidateArray[canTotalCnt].motion_vector_xl0,
                        candidateArray[canTotalCnt].motion_vector_yl0,
                        0, 0,
                        &candidateArray[canTotalCnt].drl_index,
                        bestPredmv);

                    candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                    candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
                    INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
                    context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
                    context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
                    context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] = to_inject_ref_type;

                    ++context_ptr->injected_mv_count_l0;
                }
            }
        }
    }

    // (8 Best_L1 neighbors)
    for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index) {
        const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
        const uint8_t inter_direction = me_block_results_ptr->direction;
        const uint8_t list1_ref_index = me_block_results_ptr->ref_idx_l1;
        if (inter_direction == 1) {
            for (bipredIndex = 0; bipredIndex < BIPRED_3x3_REFINEMENT_POSITIONS; ++bipredIndex) {
                if (isCompoundEnabled) {
                    /**************
                    NEWMV L1
                    ************* */
                    if (context_ptr->unipred3x3_injection >= 2) {
                        if (ALLOW_REFINEMENT_FLAG[bipredIndex] == 0)
                            continue;
                    }
                    int16_t to_inject_mv_x = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1) + BIPRED_3x3_X_POS[bipredIndex])  : ((me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2) + list1_ref_index].x_mv << 1) + BIPRED_3x3_X_POS[bipredIndex]);
                    int16_t to_inject_mv_y = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1) + BIPRED_3x3_Y_POS[bipredIndex])  : ((me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2) + list1_ref_index].y_mv << 1) + BIPRED_3x3_Y_POS[bipredIndex]);
                    uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
                    if (context_ptr->injected_mv_count_l1 == 0 || mrp_is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE) {
                        candidateArray[canTotalCnt].type = INTER_MODE;
                        candidateArray[canTotalCnt].distortion_ready = 0;
                        candidateArray[canTotalCnt].use_intrabc = 0;
                        candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                        candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)1;
                        candidateArray[canTotalCnt].inter_mode = NEWMV;
                        candidateArray[canTotalCnt].pred_mode = NEWMV;
                        candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                        candidateArray[canTotalCnt].is_compound = 0;
                        candidateArray[canTotalCnt].is_new_mv = 1;
                        candidateArray[canTotalCnt].is_zero_mv = 0;
                        candidateArray[canTotalCnt].drl_index = 0;
                        // Set the MV to ME result
                        candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x;
                        candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y;
                        // will be needed later by the rate estimation
                        candidateArray[canTotalCnt].ref_mv_index = 0;
                        candidateArray[canTotalCnt].pred_mv_weight = 0;
                        candidateArray[canTotalCnt].ref_frame_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
                        candidateArray[canTotalCnt].ref_frame_index_l0 = -1;
                        candidateArray[canTotalCnt].ref_frame_index_l1 = list1_ref_index;
                        candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                        candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

                        ChooseBestAv1MvPred(
                            context_ptr,
                            candidateArray[canTotalCnt].md_rate_estimation_ptr,
                            context_ptr->cu_ptr,
                            candidateArray[canTotalCnt].ref_frame_type,
                            candidateArray[canTotalCnt].is_compound,
                            candidateArray[canTotalCnt].pred_mode,
                            candidateArray[canTotalCnt].motion_vector_xl1,
                            candidateArray[canTotalCnt].motion_vector_yl1,
                            0, 0,
                            &candidateArray[canTotalCnt].drl_index,
                            bestPredmv);

                        candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[0].as_mv.col;
                        candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[0].as_mv.row;
                        INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
                        context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_x;
                        context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_y;
                        context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] = to_inject_ref_type;
                        ++context_ptr->injected_mv_count_l1;
                    }
                }
            }
        }
    }
    // update the total number of candidates injected
    (*candidateTotalCnt) = canTotalCnt;
    return;
}

void eighth_pel_bipred_refinement(
    const SequenceControlSet *sequence_control_set_ptr,
    PictureControlSet        *picture_control_set_ptr,
    ModeDecisionContext      *context_ptr,
    uint32_t                  me_sb_addr,
    SsMeContext              *inloop_me_context,
    EbBool                    use_close_loop_me,
    uint32_t                  close_loop_me_index,
    uint32_t                 *candidateTotalCnt) {
    uint32_t                   bipredIndex;
    uint32_t                   canTotalCnt = (*candidateTotalCnt);
    const MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr];
    uint8_t total_me_cnt = me_results->total_me_candidate_index[context_ptr->me_block_offset];
    const MeCandidate *me_block_results = me_results->me_candidate[context_ptr->me_block_offset];
    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
    EbBool isCompoundEnabled = (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
    IntMv  bestPredmv[2] = { {0}, {0} };
    MvReferenceFrame rf[2];
    if (isCompoundEnabled) {
        /**************
       NEW_NEWMV
       ************* */
        for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index) {
            const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
            const uint8_t inter_direction = me_block_results_ptr->direction;
            const uint8_t list0_ref_index = me_block_results_ptr->ref_idx_l0;
            const uint8_t list1_ref_index = me_block_results_ptr->ref_idx_l1;

            if (inter_direction == 2) {
       // (Best_L0, 8 Best_L1 neighbors)
                for (bipredIndex = 0; bipredIndex < BIPRED_3x3_REFINEMENT_POSITIONS; ++bipredIndex) {
                    if (context_ptr->bipred3x3_injection >= 2) {
                        if (ALLOW_REFINEMENT_FLAG[bipredIndex] == 0)
                            continue;
                    }
                    int16_t to_inject_mv_x_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].x_mv << 1;
                    int16_t to_inject_mv_y_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].y_mv << 1;
                    int16_t to_inject_mv_x_l1 = use_close_loop_me ? (((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1) + BIPRED_3x3_X_POS[bipredIndex])) : ((me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].x_mv << 1) + BIPRED_3x3_X_POS[bipredIndex]);
                    int16_t to_inject_mv_y_l1 = use_close_loop_me ? (((inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1) + BIPRED_3x3_Y_POS[bipredIndex])) : ((me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].y_mv << 1) + BIPRED_3x3_Y_POS[bipredIndex]);
                    rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
                    rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
                    uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
        if (context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, to_inject_ref_type) == EB_FALSE) {
                candidateArray[canTotalCnt].type = INTER_MODE;
                candidateArray[canTotalCnt].distortion_ready = 0;
                candidateArray[canTotalCnt].use_intrabc = 0;
                candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                candidateArray[canTotalCnt].is_new_mv = 1;
                candidateArray[canTotalCnt].is_zero_mv = 0;
            candidateArray[canTotalCnt].drl_index = 0;
            // Set the MV to ME result
            candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x_l0;
            candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y_l0;
            candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x_l1;
            candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y_l1;
            // will be needed later by the rate estimation
            candidateArray[canTotalCnt].ref_mv_index = 0;
            candidateArray[canTotalCnt].pred_mv_weight = 0;
            candidateArray[canTotalCnt].inter_mode = NEW_NEWMV;
            candidateArray[canTotalCnt].pred_mode = NEW_NEWMV;
            candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
            candidateArray[canTotalCnt].is_compound = 1;
            candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)2;
            rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
            rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
            candidateArray[canTotalCnt].ref_frame_type = av1_ref_frame_type(rf);
            candidateArray[canTotalCnt].ref_frame_index_l0 = list0_ref_index;
            candidateArray[canTotalCnt].ref_frame_index_l1 = list1_ref_index;
            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

            ChooseBestAv1MvPred(
                context_ptr,
                candidateArray[canTotalCnt].md_rate_estimation_ptr,
                context_ptr->cu_ptr,
                candidateArray[canTotalCnt].ref_frame_type,
                candidateArray[canTotalCnt].is_compound,
                candidateArray[canTotalCnt].pred_mode,
                candidateArray[canTotalCnt].motion_vector_xl0,
                candidateArray[canTotalCnt].motion_vector_yl0,
                candidateArray[canTotalCnt].motion_vector_xl1,
                candidateArray[canTotalCnt].motion_vector_yl1,
                &candidateArray[canTotalCnt].drl_index,
                bestPredmv);

            candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
            candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
            candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[1].as_mv.col;
            candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[1].as_mv.row;
            INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
            context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
            context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
            context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
            context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
            context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
            ++context_ptr->injected_mv_count_bipred;
                }
        }

        // (8 Best_L0 neighbors, Best_L1) :
                for (bipredIndex = 0; bipredIndex < BIPRED_3x3_REFINEMENT_POSITIONS; ++bipredIndex)
        {
                    if (context_ptr->bipred3x3_injection >= 2) {
                if (ALLOW_REFINEMENT_FLAG[bipredIndex] == 0)
                    continue;
            }
                    int16_t to_inject_mv_x_l0 = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] << 1) + BIPRED_3x3_X_POS[bipredIndex]) : ((me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].x_mv << 1) + BIPRED_3x3_X_POS[bipredIndex]);
                    int16_t to_inject_mv_y_l0 = use_close_loop_me ? ((inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] << 1) + BIPRED_3x3_Y_POS[bipredIndex]) : ((me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].y_mv << 1) + BIPRED_3x3_Y_POS[bipredIndex]);
            int16_t to_inject_mv_x_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].x_mv << 1;
            int16_t to_inject_mv_y_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].y_mv << 1;
            rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
            rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
            uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
            if (context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, to_inject_ref_type) == EB_FALSE) {
                candidateArray[canTotalCnt].type = INTER_MODE;
                candidateArray[canTotalCnt].distortion_ready = 0;
                candidateArray[canTotalCnt].use_intrabc = 0;
                candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                candidateArray[canTotalCnt].is_new_mv = 1;
                candidateArray[canTotalCnt].is_zero_mv = 0;
                candidateArray[canTotalCnt].drl_index = 0;
                // Set the MV to ME result
                candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x_l0;
                candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y_l0;
                candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x_l1;
                candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y_l1;
                // will be needed later by the rate estimation
                candidateArray[canTotalCnt].ref_mv_index = 0;
                candidateArray[canTotalCnt].pred_mv_weight = 0;
                candidateArray[canTotalCnt].inter_mode = NEW_NEWMV;
                candidateArray[canTotalCnt].pred_mode = NEW_NEWMV;
                candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                candidateArray[canTotalCnt].is_compound = 1;
                candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)2;
                rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
                rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
                candidateArray[canTotalCnt].ref_frame_type = av1_ref_frame_type(rf);
                candidateArray[canTotalCnt].ref_frame_index_l0 = list0_ref_index;
                candidateArray[canTotalCnt].ref_frame_index_l1 = list1_ref_index;
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

                ChooseBestAv1MvPred(
                    context_ptr,
                    candidateArray[canTotalCnt].md_rate_estimation_ptr,
                    context_ptr->cu_ptr,
                    candidateArray[canTotalCnt].ref_frame_type,
                    candidateArray[canTotalCnt].is_compound,
                    candidateArray[canTotalCnt].pred_mode,
                    candidateArray[canTotalCnt].motion_vector_xl0,
                    candidateArray[canTotalCnt].motion_vector_yl0,
                    candidateArray[canTotalCnt].motion_vector_xl1,
                    candidateArray[canTotalCnt].motion_vector_yl1,
                    &candidateArray[canTotalCnt].drl_index,
                    bestPredmv);

                candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
                candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[1].as_mv.col;
                candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[1].as_mv.row;
                INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
                context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
                ++context_ptr->injected_mv_count_bipred;
                }
            }
        }
            }
        }
    // update the total number of candidates injected
    (*candidateTotalCnt) = canTotalCnt;
    return;
}

#endif

uint8_t GetMaxDrlIndex(uint8_t  refmvCnt, PredictionMode   mode)
{
    uint8_t maxDrl = 0;

    if (mode == NEWMV || mode == NEW_NEWMV) {
        if (refmvCnt < 2)
            maxDrl = 1;
        else if (refmvCnt == 2)
            maxDrl = 2;
        else
            maxDrl = 3;
    }

    if (mode == NEARMV || mode == NEAR_NEARMV || mode == NEAR_NEWMV || mode == NEW_NEARMV) {
        if (refmvCnt < 3)
            maxDrl = 1;
        else if (refmvCnt == 3)
            maxDrl = 2;
        else
            maxDrl = 3;
    }

    return maxDrl;
}
/*********************************************************************
**********************************************************************
        Upto 12 inter Candidated injected
        Min 6 inter Candidated injected
UniPred L0 : NEARST         + upto 3x NEAR
UniPred L1 : NEARST         + upto 3x NEAR
BIPred     : NEARST_NEARST  + upto 3x NEAR_NEAR
**********************************************************************
**********************************************************************/
#if INJ_MVP
#if !II_COMP
void av1_set_ref_frame(MvReferenceFrame *rf,
    int8_t ref_frame_type);
#endif
void inject_mvp_candidates_II(
    struct ModeDecisionContext     *context_ptr,
    PictureControlSet              *picture_control_set_ptr,
    CodingUnit                     *cu_ptr,
    MvReferenceFrame                 ref_pair,
    uint32_t                         *candTotCnt)
{
    EbBool allow_compound = (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE || context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) ? EB_FALSE : EB_TRUE;
    uint8_t inj_mv;
    uint32_t                   canIdx = *candTotCnt;
    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
    MacroBlockD  *xd = cu_ptr->av1xd;
    uint8_t drli, maxDrlIndex;
    IntMv    nearestmv[2], nearmv[2], ref_mv[2];

    MvReferenceFrame rf[2];
    av1_set_ref_frame(rf, ref_pair);
#if COMP_MODE

    BlockSize bsize = context_ptr->blk_geom->bsize;                       // bloc size
    MD_COMP_TYPE compound_types_to_try = picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;
    MD_COMP_TYPE cur_type; //BIP 3x3 MiSize >= BLOCK_8X8 && MiSize <= BLOCK_32X32)
    MD_COMP_TYPE tot_comp_types = (bsize >= BLOCK_8X8 && bsize<= BLOCK_32X32 ) ? compound_types_to_try :
                                  (compound_types_to_try == MD_COMP_WEDGE )? MD_COMP_DIFF0 :
                                   picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;//MD_COMP_DIST;// MD_COMP_AVG;//
#endif

    //single ref/list
    if (rf[1] == NONE_FRAME)
    {
        MvReferenceFrame frame_type = rf[0];
        uint8_t list_idx = get_list_idx(rf[0]);
        uint8_t ref_idx = get_ref_frame_idx(rf[0]);

        //NEAREST
        int16_t to_inject_mv_x = context_ptr->cu_ptr->ref_mvs[frame_type][0].as_mv.col;
        int16_t to_inject_mv_y = context_ptr->cu_ptr->ref_mvs[frame_type][0].as_mv.row;

        inj_mv = list_idx == 0 ?
            context_ptr->injected_mv_count_l0 == 0 || mrp_is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y, frame_type) == EB_FALSE :
            context_ptr->injected_mv_count_l1 == 0 || mrp_is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y, frame_type) == EB_FALSE;

        if (inj_mv) {

#if II_SEARCH // NEARESTMV
            uint8_t tot_ii_types = svt_is_interintra_allowed(picture_control_set_ptr->parent_pcs_ptr->enable_inter_intra,bsize, NEARESTMV, rf) ? II_COUNT : 1;
            uint8_t ii_type;
            for (ii_type = 0; ii_type < tot_ii_types; ii_type++)
            {
#endif
            candidateArray[canIdx].type = INTER_MODE;
            candidateArray[canIdx].inter_mode = NEARESTMV;
            candidateArray[canIdx].pred_mode = NEARESTMV;
            candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
            candidateArray[canIdx].is_compound = 0;
            candidateArray[canIdx].distortion_ready = 0;
            candidateArray[canIdx].use_intrabc = 0;
            candidateArray[canIdx].merge_flag = EB_FALSE;
            candidateArray[canIdx].prediction_direction[0] = list_idx;
            candidateArray[canIdx].is_new_mv = 0;
            candidateArray[canIdx].is_zero_mv = 0;

            candidateArray[canIdx].drl_index = 0;
            candidateArray[canIdx].ref_mv_index = 0;
            candidateArray[canIdx].pred_mv_weight = 0;
            candidateArray[canIdx].ref_frame_type = frame_type;

            candidateArray[canIdx].ref_frame_index_l0 = (list_idx == 0) ? ref_idx : -1;
            candidateArray[canIdx].ref_frame_index_l1 = (list_idx == 1) ? ref_idx : -1;
#if ATB_TX_TYPE_SUPPORT_PER_TU
            candidateArray[canIdx].transform_type[0] = DCT_DCT;
            candidateArray[canIdx].transform_type_uv = DCT_DCT;
#else
            candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
            candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif
            if (list_idx == 0) {
                candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x;
                candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y;
                context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
                context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
                context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] = frame_type;
                ++context_ptr->injected_mv_count_l0;
            }
            else {
                candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x;
                candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y;
                context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_x;
                context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_y;
                context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] = frame_type;
                ++context_ptr->injected_mv_count_l1;
            }
#if II_SEARCH
                candidateArray[canIdx].is_interintra_used = ii_type == 0 ? 0 : 1;
                if (ii_type == 1) {
                    inter_intra_search(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canIdx]);

                    candidateArray[canIdx].use_wedge_interintra = 1;
                    candidateArray[canIdx].ii_wedge_sign = 0;

                }
                if (ii_type == 2)// smooth
                {
                    candidateArray[canIdx].interintra_mode = candidateArray[canIdx-1].interintra_mode;
                    candidateArray[canIdx].use_wedge_interintra = 0;
                }
#endif
            INCRMENT_CAND_TOTAL_COUNT(canIdx);
#if II_SEARCH
            }
#endif
        }

        //NEAR
        maxDrlIndex = GetMaxDrlIndex(xd->ref_mv_count[frame_type], NEARMV);

        for (drli = 0; drli < maxDrlIndex; drli++)
        {
            get_av1_mv_pred_drl(
                context_ptr,
                cu_ptr,
                frame_type,
                0,
                NEARMV,
                drli,
                nearestmv,
                nearmv,
                ref_mv);

            int16_t to_inject_mv_x = nearmv[0].as_mv.col;
            int16_t to_inject_mv_y = nearmv[0].as_mv.row;

            inj_mv = list_idx == 0 ?
                context_ptr->injected_mv_count_l0 == 0 || mrp_is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y, frame_type) == EB_FALSE :
                context_ptr->injected_mv_count_l1 == 0 || mrp_is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y, frame_type) == EB_FALSE;

            if (inj_mv) {
#if II_SEARCH // NEARMV
            uint8_t tot_ii_types = svt_is_interintra_allowed(picture_control_set_ptr->parent_pcs_ptr->enable_inter_intra,bsize, NEARMV, rf) ? II_COUNT : 1;
            uint8_t ii_type;
            for (ii_type = 0; ii_type < tot_ii_types; ii_type++)
            {
#endif
                candidateArray[canIdx].type = INTER_MODE;
                candidateArray[canIdx].inter_mode = NEARMV;
                candidateArray[canIdx].pred_mode = NEARMV;
                candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
                candidateArray[canIdx].is_compound = 0;
                candidateArray[canIdx].distortion_ready = 0;
                candidateArray[canIdx].use_intrabc = 0;
                candidateArray[canIdx].merge_flag = EB_FALSE;
                candidateArray[canIdx].prediction_direction[0] = list_idx;
                candidateArray[canIdx].is_new_mv = 0;
                candidateArray[canIdx].is_zero_mv = 0;
                candidateArray[canIdx].drl_index = drli;
                candidateArray[canIdx].ref_mv_index = 0;
                candidateArray[canIdx].pred_mv_weight = 0;
                candidateArray[canIdx].ref_frame_type = frame_type;

                candidateArray[canIdx].ref_frame_index_l0 = (list_idx == 0) ? ref_idx : -1;
                candidateArray[canIdx].ref_frame_index_l1 = (list_idx == 1) ? ref_idx : -1;

#if ATB_TX_TYPE_SUPPORT_PER_TU
                candidateArray[canIdx].transform_type[0] = DCT_DCT;
                candidateArray[canIdx].transform_type_uv = DCT_DCT;
#else
                candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif
                if (list_idx == 0) {
                    candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x;
                    candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y;
                    context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
                    context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
                    context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] = frame_type;
                    ++context_ptr->injected_mv_count_l0;
                }
                else {
                    candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x;
                    candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y;
                    context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_x;
                    context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_y;
                    context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] = frame_type;
                    ++context_ptr->injected_mv_count_l1;
                }
#if II_SEARCH
                candidateArray[canIdx].is_interintra_used = ii_type == 0 ? 0 : 1;
                if (ii_type == 1) {
                    inter_intra_search(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canIdx]);

                    candidateArray[canIdx].use_wedge_interintra = 1;
                    candidateArray[canIdx].ii_wedge_sign = 0;

                }
                if (ii_type == 2)// smooth
                {
                    candidateArray[canIdx].interintra_mode = candidateArray[canIdx-1].interintra_mode;
                    candidateArray[canIdx].use_wedge_interintra = 0;
                }
#endif

                INCRMENT_CAND_TOTAL_COUNT(canIdx);
#if II_SEARCH
            }
#endif
            }
        }
    }
    else if (allow_compound)
    {
        uint8_t ref_idx_0 = get_ref_frame_idx(rf[0]);
        uint8_t ref_idx_1 = get_ref_frame_idx(rf[1]);

#if !MRP_MD_UNI_DIR_BIPRED
        if (list_idx_0 != list_idx_1) //only bi-directional compound for now
#endif
        {
            //NEAREST_NEAREST
            int16_t to_inject_mv_x_l0 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[ref_pair][0].this_mv.as_mv.col;
            int16_t to_inject_mv_y_l0 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[ref_pair][0].this_mv.as_mv.row;
            int16_t to_inject_mv_x_l1 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[ref_pair][0].comp_mv.as_mv.col;
            int16_t to_inject_mv_y_l1 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[ref_pair][0].comp_mv.as_mv.row;

            inj_mv = context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, ref_pair) == EB_FALSE;

            if (inj_mv) {
#if COMP_MODE


                context_ptr->variance_ready = 0 ;
                    for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types ; cur_type++)
                {
                // If two predictors are very similar, skip wedge compound mode search
                if (context_ptr->variance_ready)
                    if (context_ptr->prediction_mse < 8 || (!have_newmv_in_inter_mode(NEAREST_NEARESTMV) && context_ptr->prediction_mse  < 64))
                        continue;

#if ! COMP_DIFF
                    //NEAREST-NEAREST
                    determine_compound_mode(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canIdx],
                        cur_type);
#endif
#endif
                candidateArray[canIdx].type = INTER_MODE;
                candidateArray[canIdx].inter_mode = NEAREST_NEARESTMV;
                candidateArray[canIdx].pred_mode = NEAREST_NEARESTMV;
                candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
                candidateArray[canIdx].is_compound = 1;
#if II_COMP
                candidateArray[canIdx].is_interintra_used = 0;
#endif
                candidateArray[canIdx].distortion_ready = 0;
                candidateArray[canIdx].use_intrabc = 0;

                candidateArray[canIdx].merge_flag =
#if COMP_MODE
                    cur_type == MD_COMP_AVG &&
#endif
                    picture_control_set_ptr->parent_pcs_ptr->is_skip_mode_allowed &&
                    (rf[0] == picture_control_set_ptr->parent_pcs_ptr->skip_mode_info.ref_frame_idx_0 + 1) &&
                    (rf[1] == picture_control_set_ptr->parent_pcs_ptr->skip_mode_info.ref_frame_idx_1 + 1) ? EB_TRUE : EB_FALSE;

                candidateArray[canIdx].prediction_direction[0] = BI_PRED;
                candidateArray[canIdx].is_new_mv = 0;
                candidateArray[canIdx].is_zero_mv = 0;
                candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x_l0;
                candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y_l0;
                candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x_l1;
                candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y_l1;
                candidateArray[canIdx].drl_index = 0;
                candidateArray[canIdx].ref_mv_index = 0;
                candidateArray[canIdx].pred_mv_weight = 0;
                candidateArray[canIdx].ref_frame_type = ref_pair;
                candidateArray[canIdx].ref_frame_index_l0 = ref_idx_0;
                candidateArray[canIdx].ref_frame_index_l1 = ref_idx_1;

#if ATB_TX_TYPE_SUPPORT_PER_TU
                candidateArray[canIdx].transform_type[0] = DCT_DCT;
                candidateArray[canIdx].transform_type_uv = DCT_DCT;
#else
                candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

#if COMP_DIFF
                //NEAREST-NEAREST
                determine_compound_mode(
                    picture_control_set_ptr,
                    context_ptr,
                    &candidateArray[canIdx],
                    cur_type);
#endif
                context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = ref_pair;
                ++context_ptr->injected_mv_count_bipred;

                INCRMENT_CAND_TOTAL_COUNT(canIdx);
#if COMP_MODE
                }
#endif
            }

            //NEAR_NEAR
            maxDrlIndex = GetMaxDrlIndex(xd->ref_mv_count[ref_pair], NEAR_NEARMV);
            for (drli = 0; drli < maxDrlIndex; drli++) {
                get_av1_mv_pred_drl(
                    context_ptr,
                    cu_ptr,
                    ref_pair,
                    1,
                    NEAR_NEARMV,
                    drli,
                    nearestmv,
                    nearmv,
                    ref_mv);

                int16_t to_inject_mv_x_l0 = nearmv[0].as_mv.col;
                int16_t to_inject_mv_y_l0 = nearmv[0].as_mv.row;
                int16_t to_inject_mv_x_l1 = nearmv[1].as_mv.col;
                int16_t to_inject_mv_y_l1 = nearmv[1].as_mv.row;

                inj_mv = context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, ref_pair) == EB_FALSE;

                if (inj_mv) {
#if COMP_MODE

                context_ptr->variance_ready = 0 ;
                    for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types ; cur_type++)
                    {
                // If two predictors are very similar, skip wedge compound mode search
                if (context_ptr->variance_ready)
                    if (context_ptr->prediction_mse < 8 || (!have_newmv_in_inter_mode(NEAR_NEARMV) && context_ptr->prediction_mse  < 64))
                        continue;

#endif
                    candidateArray[canIdx].type = INTER_MODE;
                    candidateArray[canIdx].inter_mode = NEAR_NEARMV;
                    candidateArray[canIdx].pred_mode = NEAR_NEARMV;
                    candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
                    candidateArray[canIdx].is_compound = 1;
#if II_COMP
                    candidateArray[canIdx].is_interintra_used = 0;
#endif
                    candidateArray[canIdx].distortion_ready = 0;
                    candidateArray[canIdx].use_intrabc = 0;
                    candidateArray[canIdx].merge_flag = EB_FALSE;
                    candidateArray[canIdx].prediction_direction[0] = BI_PRED;
                    candidateArray[canIdx].is_new_mv = 0;
                    candidateArray[canIdx].is_zero_mv = 0;

                    candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x_l0;
                    candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y_l0;
                    candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x_l1;
                    candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y_l1;

                    candidateArray[canIdx].drl_index = drli;
                    candidateArray[canIdx].ref_mv_index = 0;
                    candidateArray[canIdx].pred_mv_weight = 0;

                    candidateArray[canIdx].ref_frame_type = ref_pair;

                    candidateArray[canIdx].ref_frame_index_l0 = ref_idx_0;
                    candidateArray[canIdx].ref_frame_index_l1 = ref_idx_1;

#if ATB_TX_TYPE_SUPPORT_PER_TU
                    candidateArray[canIdx].transform_type[0] = DCT_DCT;
                    candidateArray[canIdx].transform_type_uv = DCT_DCT;
#else
                    candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                    candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

                    context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                    context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                    context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                    context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                    context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = ref_pair;
                    ++context_ptr->injected_mv_count_bipred;
#if COMP_MODE
                    //NEAR-NEAR
                    determine_compound_mode(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canIdx],
                        cur_type);
#endif
                    INCRMENT_CAND_TOTAL_COUNT(canIdx);

#if COMP_MODE
                    }
#endif
                }
            }
        }
    }

    //update tot Candidate count
    *candTotCnt = canIdx;
}
#else

void InjectAv1MvpCandidates(
    struct ModeDecisionContext     *context_ptr,
    CodingUnit                     *cu_ptr,
    MvReferenceFrame               *ref_frames,
    PictureControlSet              *picture_control_set_ptr,
    uint32_t                            lcuAddr,
#if !M8_SKIP_BLK
    uint32_t                            leaf_index,
#endif
    EbBool                           allow_bipred,
    uint32_t                           *candTotCnt)
{
//MRP_LIST_REF_IDX_TYPE_LT
//MRP_LIST_REF_IDX_TYPE_LT
//MRP_LIST_REF_IDX_TYPE_LT
//MRP_LIST_REF_IDX_TYPE_LT
//MRP_LIST_REF_IDX_TYPE_LT
//MRP_LIST_REF_IDX_TYPE_LT
    // CHECK_CAND  (x5)
    // MRP_DUPLICATION_FIX (x12)
#if !M8_SKIP_BLK
    (void)leaf_index;
#endif
    (void)lcuAddr;
    (void)ref_frames;
    uint32_t                   canIdx = *candTotCnt;
    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
    EbBool isCompoundEnabled = (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
#if  !BASE_LAYER_REF && !MRP_REF_MODE
    isCompoundEnabled = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) ? EB_FALSE : isCompoundEnabled;
#endif
    MacroBlockD  *xd = cu_ptr->av1xd;
    uint8_t drli, maxDrlIndex;
    IntMv    nearestmv[2], nearmv[2], ref_mv[2];

    //NEAREST_L0
    int16_t to_inject_mv_x = context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.col;
    int16_t to_inject_mv_y = context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.row;
    if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
    candidateArray[canIdx].type = INTER_MODE;
    candidateArray[canIdx].inter_mode = NEARESTMV;
    candidateArray[canIdx].pred_mode = NEARESTMV;
    candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
    candidateArray[canIdx].is_compound = 0;
    candidateArray[canIdx].distortion_ready = 0;
    candidateArray[canIdx].use_intrabc = 0;
    candidateArray[canIdx].merge_flag = EB_FALSE;
    candidateArray[canIdx].prediction_direction[0] = UNI_PRED_LIST_0;
    candidateArray[canIdx].is_new_mv = 0;
    candidateArray[canIdx].is_zero_mv = 0;
    candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x;
    candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y;
    candidateArray[canIdx].drl_index = 0;
    candidateArray[canIdx].ref_mv_index = 0;
    candidateArray[canIdx].pred_mv_weight = 0;
    candidateArray[canIdx].ref_frame_type = LAST_FRAME;
    candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
    candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
    ++canIdx;
    context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
    context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
    ++context_ptr->injected_mv_count_l0;
    }
#if M9_NEAR_INJECTION
    if (context_ptr->near_mv_injection) {
#endif
    //NEAR_L0
    maxDrlIndex = GetMaxDrlIndex(xd->ref_mv_count[LAST_FRAME], NEARMV);
    //maxDrlIndex = 1;
    for (drli = 0; drli < maxDrlIndex; drli++) {
        get_av1_mv_pred_drl(
            context_ptr,
            cu_ptr,
            LAST_FRAME,
            0,
            NEARMV,
            drli,
            nearestmv,
            nearmv,
            ref_mv);

        int16_t to_inject_mv_x = nearmv[0].as_mv.col;
        int16_t to_inject_mv_y = nearmv[0].as_mv.row;
        if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
        candidateArray[canIdx].type = INTER_MODE;
        candidateArray[canIdx].inter_mode = NEARMV;
        candidateArray[canIdx].pred_mode = NEARMV;
        candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
        candidateArray[canIdx].is_compound = 0;
        candidateArray[canIdx].distortion_ready = 0;
        candidateArray[canIdx].use_intrabc = 0;
        candidateArray[canIdx].merge_flag = EB_FALSE;
        candidateArray[canIdx].prediction_direction[0] = UNI_PRED_LIST_0;
        candidateArray[canIdx].is_new_mv = 0;
        candidateArray[canIdx].is_zero_mv = 0;
        candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x;
        candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y;
        candidateArray[canIdx].drl_index = drli;
        candidateArray[canIdx].ref_mv_index = 0;
        candidateArray[canIdx].pred_mv_weight = 0;
        candidateArray[canIdx].ref_frame_type = LAST_FRAME;
        candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
        candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
        ++canIdx;
        context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
        context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
        ++context_ptr->injected_mv_count_l0;
        }
    }
#if M9_NEAR_INJECTION
    }
#endif

    if (isCompoundEnabled) {
        int16_t to_inject_mv_x = context_ptr->cu_ptr->ref_mvs[BWDREF_FRAME][0].as_mv.col;
        int16_t to_inject_mv_y = context_ptr->cu_ptr->ref_mvs[BWDREF_FRAME][0].as_mv.row;
        if (context_ptr->injected_mv_count_l1 == 0 || is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
        //NEAREST_L1
        candidateArray[canIdx].type = INTER_MODE;
        candidateArray[canIdx].inter_mode = NEARESTMV;
        candidateArray[canIdx].pred_mode = NEARESTMV;
        candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
        candidateArray[canIdx].is_compound = 0;
        candidateArray[canIdx].distortion_ready = 0;
        candidateArray[canIdx].use_intrabc = 0;
        candidateArray[canIdx].merge_flag = EB_FALSE;
        candidateArray[canIdx].prediction_direction[0] = UNI_PRED_LIST_1;
        candidateArray[canIdx].is_new_mv = 0;
        candidateArray[canIdx].is_zero_mv = 0;
        candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x;
        candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y;
        candidateArray[canIdx].drl_index = 0;
        candidateArray[canIdx].ref_mv_index = 0;
        candidateArray[canIdx].pred_mv_weight = 0;
        candidateArray[canIdx].ref_frame_type = BWDREF_FRAME;
        candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
        candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
        ++canIdx;
        context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_x;
        context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_y;
        ++context_ptr->injected_mv_count_l1;
    }
#if M9_NEAR_INJECTION
        if (context_ptr->near_mv_injection) {
#endif
        //NEAR_L1
        maxDrlIndex = GetMaxDrlIndex(xd->ref_mv_count[BWDREF_FRAME], NEARMV);

        for (drli = 0; drli < maxDrlIndex; drli++) {
            get_av1_mv_pred_drl(
                context_ptr,
                cu_ptr,
                BWDREF_FRAME,
                0,
                NEARMV,
                drli,
                nearestmv,
                nearmv,
                ref_mv);
            int16_t to_inject_mv_x = nearmv[0].as_mv.col;
            int16_t to_inject_mv_y = nearmv[0].as_mv.row;
            if (context_ptr->injected_mv_count_l1 == 0 || is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
            candidateArray[canIdx].type = INTER_MODE;
            candidateArray[canIdx].inter_mode = NEARMV;
            candidateArray[canIdx].pred_mode = NEARMV;
            candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
            candidateArray[canIdx].is_compound = 0;
            candidateArray[canIdx].distortion_ready = 0;
            candidateArray[canIdx].use_intrabc = 0;
            candidateArray[canIdx].merge_flag = EB_FALSE;
            candidateArray[canIdx].prediction_direction[0] = UNI_PRED_LIST_1;
            candidateArray[canIdx].is_new_mv = 0;
            candidateArray[canIdx].is_zero_mv = 0;
            candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x;
            candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y;
            candidateArray[canIdx].drl_index = drli;
            candidateArray[canIdx].ref_mv_index = 0;
            candidateArray[canIdx].pred_mv_weight = 0;
            candidateArray[canIdx].ref_frame_type = BWDREF_FRAME;
            candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
            candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
            ++canIdx;
            context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_x;
            context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_y;
            ++context_ptr->injected_mv_count_l1;
            }
        }
#if M9_NEAR_INJECTION
        }
#endif

#if BASE_LAYER_REF || MRP_REF_MODE
        if (allow_bipred)
#endif
        {
            //SKIP (NEAREST_NEAREST with LAST_BWD_FRAME)
            int16_t to_inject_mv_x_l0 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[LAST_BWD_FRAME][0].this_mv.as_mv.col;
            int16_t to_inject_mv_y_l0 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[LAST_BWD_FRAME][0].this_mv.as_mv.row;
            int16_t to_inject_mv_x_l1 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[LAST_BWD_FRAME][0].comp_mv.as_mv.col;
            int16_t to_inject_mv_y_l1 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[LAST_BWD_FRAME][0].comp_mv.as_mv.row;
            if (context_ptr->injected_mv_count_bipred == 0 || is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1) == EB_FALSE) {
                candidateArray[canIdx].type = INTER_MODE;
                candidateArray[canIdx].inter_mode = NEAREST_NEARESTMV;
                candidateArray[canIdx].pred_mode = NEAREST_NEARESTMV;
                candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
                candidateArray[canIdx].is_compound = 1;
                candidateArray[canIdx].distortion_ready = 0;
                candidateArray[canIdx].use_intrabc = 0;
                candidateArray[canIdx].merge_flag = EB_TRUE;
                candidateArray[canIdx].prediction_direction[0] = BI_PRED;
                candidateArray[canIdx].is_new_mv = 0;
                candidateArray[canIdx].is_zero_mv = 0;
                candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x_l0;
                candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y_l0;
                candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x_l1;
                candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y_l1;
                candidateArray[canIdx].drl_index = 0;
                candidateArray[canIdx].ref_mv_index = 0;
                candidateArray[canIdx].pred_mv_weight = 0;
                candidateArray[canIdx].ref_frame_type = LAST_BWD_FRAME;
                candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
                ++canIdx;
                context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                ++context_ptr->injected_mv_count_bipred;
            }
        }
#if M9_NEAR_INJECTION
        if (context_ptr->near_mv_injection) {
#endif
        //NEAR_NEAR
        if (allow_bipred) {
            maxDrlIndex = GetMaxDrlIndex(xd->ref_mv_count[LAST_BWD_FRAME], NEAR_NEARMV);
            //maxDrlIndex = 1;
            for (drli = 0; drli < maxDrlIndex; drli++) {
                get_av1_mv_pred_drl(
                    context_ptr,
                    cu_ptr,
                    LAST_BWD_FRAME,
                    1,
                    NEAR_NEARMV,
                    drli,
                    nearestmv,
                    nearmv,
                    ref_mv);
                int16_t to_inject_mv_x_l0 = nearmv[0].as_mv.col;
                int16_t to_inject_mv_y_l0 = nearmv[0].as_mv.row;
                int16_t to_inject_mv_x_l1 = nearmv[1].as_mv.col;
                int16_t to_inject_mv_y_l1 = nearmv[1].as_mv.row;
                if (context_ptr->injected_mv_count_bipred == 0 || is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1) == EB_FALSE) {
                candidateArray[canIdx].type = INTER_MODE;
                candidateArray[canIdx].inter_mode = NEAR_NEARMV;
                candidateArray[canIdx].pred_mode = NEAR_NEARMV;
                candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
                candidateArray[canIdx].is_compound = 1;
                candidateArray[canIdx].distortion_ready = 0;
                candidateArray[canIdx].use_intrabc = 0;
                candidateArray[canIdx].merge_flag = EB_FALSE;
                candidateArray[canIdx].prediction_direction[0] = BI_PRED;
                candidateArray[canIdx].is_new_mv = 0;
                candidateArray[canIdx].is_zero_mv = 0;
                candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x_l0;
                candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y_l0;
                candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x_l1;
                candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y_l1;
                candidateArray[canIdx].drl_index = drli;
                candidateArray[canIdx].ref_mv_index = 0;
                candidateArray[canIdx].pred_mv_weight = 0;
                candidateArray[canIdx].ref_frame_type = LAST_BWD_FRAME;
                candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
                ++canIdx;
                context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                ++context_ptr->injected_mv_count_bipred;
                }
            }
        }
#if M9_NEAR_INJECTION
        }
#endif
    }
    //update tot Candidate count
    *candTotCnt = canIdx;
}
#endif

#if NEW_NEAREST_NEW_INJECTION
void inject_new_nearest_new_comb_candidates(
    const SequenceControlSet       *sequence_control_set_ptr,
    struct ModeDecisionContext     *context_ptr,
    PictureControlSet              *picture_control_set_ptr,
    MvReferenceFrame                ref_pair,
    uint32_t                       *candTotCnt)
{
    uint8_t inj_mv;
    uint32_t                  canIdx = *candTotCnt;
    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
#if COMP_MODE

    BlockSize bsize = context_ptr->blk_geom->bsize;                       // bloc size
    MD_COMP_TYPE compound_types_to_try = picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;
    MD_COMP_TYPE cur_type; //BIP 3x3 MiSize >= BLOCK_8X8 && MiSize <= BLOCK_32X32)
    MD_COMP_TYPE tot_comp_types = (bsize >= BLOCK_8X8 && bsize<= BLOCK_32X32 ) ? compound_types_to_try :
                                  (compound_types_to_try == MD_COMP_WEDGE )? MD_COMP_DIFF0 :
                                   picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;//MD_COMP_DIST;// MD_COMP_AVG;//
#endif
#if NEW_NEAR_FIX
    MacroBlockD  *xd = context_ptr->cu_ptr->av1xd;
    uint8_t drli, maxDrlIndex;
    IntMv    nearestmv[2], nearmv[2], ref_mv[2];
#endif
    MvReferenceFrame rf[2];
    av1_set_ref_frame(rf, ref_pair);

    {
        uint8_t ref_idx_0 = get_ref_frame_idx(rf[0]);
        uint8_t ref_idx_1 = get_ref_frame_idx(rf[1]);

        if (rf[1] != NONE_FRAME)
        {
            {
                //NEAREST_NEWMV
                const MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[context_ptr->me_sb_addr];

                int16_t to_inject_mv_x_l0 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[ref_pair][0].this_mv.as_mv.col;
                int16_t to_inject_mv_y_l0 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[ref_pair][0].this_mv.as_mv.row;
                int16_t to_inject_mv_x_l1 = me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (get_list_idx(rf[1]) << 2) : (get_list_idx(rf[1]) << 1)) + ref_idx_1].x_mv << 1;
                int16_t to_inject_mv_y_l1 = me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (get_list_idx(rf[1]) << 2) : (get_list_idx(rf[1]) << 1)) + ref_idx_1].y_mv << 1;

                inj_mv = context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, ref_pair) == EB_FALSE;

#if PRUNE_REF_FRAME_FRO_REC_PARTITION_MVP
                uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
                uint8_t skip_cand = check_ref_beackout(
                    picture_control_set_ptr,
                    context_ptr,
                    to_inject_ref_type,
                    context_ptr->blk_geom->shape);

                if (!skip_cand && (inj_mv)) {
#else
                if (inj_mv) {
#endif

#if COMP_MODE

                context_ptr->variance_ready = 0 ;
                    for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types ; cur_type++)
                    {
                // If two predictors are very similar, skip wedge compound mode search
                if (context_ptr->variance_ready)
                    if (context_ptr->prediction_mse < 8 || (!have_newmv_in_inter_mode(NEAREST_NEWMV) && context_ptr->prediction_mse  < 64))
                        continue;
#endif
                    candidateArray[canIdx].type = INTER_MODE;
                    candidateArray[canIdx].inter_mode = NEAREST_NEWMV;
                    candidateArray[canIdx].pred_mode = NEAREST_NEWMV;
                    candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
                    candidateArray[canIdx].is_compound = 1;
#if II_COMP
                    candidateArray[canIdx].is_interintra_used = 0;
#endif
                    candidateArray[canIdx].distortion_ready = 0;
                    candidateArray[canIdx].use_intrabc = 0;

                    candidateArray[canIdx].merge_flag = EB_FALSE;

                    candidateArray[canIdx].prediction_direction[0] = BI_PRED;
                    candidateArray[canIdx].is_new_mv = 0;
                    candidateArray[canIdx].is_zero_mv = 0;
                    candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x_l0;
                    candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y_l0;
                    candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x_l1;
                    candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y_l1;
                    candidateArray[canIdx].drl_index = 0;
                    candidateArray[canIdx].ref_mv_index = 0;
                    candidateArray[canIdx].pred_mv_weight = 0;
                    candidateArray[canIdx].ref_frame_type = ref_pair;
                    candidateArray[canIdx].ref_frame_index_l0 = ref_idx_0;
                    candidateArray[canIdx].ref_frame_index_l1 = ref_idx_1;
#if ATB_TX_TYPE_SUPPORT_PER_TU
                    candidateArray[canIdx].transform_type[0] = DCT_DCT;
                    candidateArray[canIdx].transform_type_uv = DCT_DCT;
#else
                    candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                    candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

#if NEW_NEAR_FIX
                    get_av1_mv_pred_drl(
                        context_ptr,
                        context_ptr->cu_ptr,
                        candidateArray[canIdx].ref_frame_type,
                        candidateArray[canIdx].is_compound,
                        NEAREST_NEWMV,
                        0,//not needed drli,
                        nearestmv,
                        nearmv,
                        ref_mv);
                    candidateArray[canIdx].motion_vector_pred_x[REF_LIST_1] = ref_mv[1].as_mv.col;
                    candidateArray[canIdx].motion_vector_pred_y[REF_LIST_1] = ref_mv[1].as_mv.row;
#else
                    IntMv  bestPredmv[2] = { {0}, {0} };

                    ChooseBestAv1MvPred(
                        context_ptr,
                        candidateArray[canIdx].md_rate_estimation_ptr,
                        context_ptr->cu_ptr,
                        candidateArray[canIdx].ref_frame_type,
                        candidateArray[canIdx].is_compound,
                        candidateArray[canIdx].pred_mode,
                        candidateArray[canIdx].motion_vector_xl0,
                        candidateArray[canIdx].motion_vector_yl0,
                        candidateArray[canIdx].motion_vector_xl1,
                        candidateArray[canIdx].motion_vector_yl1,
                        &candidateArray[canIdx].drl_index,
                        bestPredmv);

                    candidateArray[canIdx].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                    candidateArray[canIdx].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
                    candidateArray[canIdx].motion_vector_pred_x[REF_LIST_1] = bestPredmv[1].as_mv.col;
                    candidateArray[canIdx].motion_vector_pred_y[REF_LIST_1] = bestPredmv[1].as_mv.row;
#endif
                    context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                    context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                    context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                    context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                    context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = ref_pair;
                    ++context_ptr->injected_mv_count_bipred;
#if COMP_MODE
                    //NEAREST_NEW
                    determine_compound_mode(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canIdx],
                        cur_type);
#endif
                    INCRMENT_CAND_TOTAL_COUNT(canIdx);
#if COMP_MODE
                    }
#endif
                }
            }

            {
                //NEW_NEARESTMV
                const MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[context_ptr->me_sb_addr];

                int16_t to_inject_mv_x_l0 = me_results->me_mv_array[context_ptr->me_block_offset][ref_idx_0].x_mv << 1;
                int16_t to_inject_mv_y_l0 = me_results->me_mv_array[context_ptr->me_block_offset][ref_idx_0].y_mv << 1;
                int16_t to_inject_mv_x_l1 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[ref_pair][0].comp_mv.as_mv.col;
                int16_t to_inject_mv_y_l1 = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[ref_pair][0].comp_mv.as_mv.row;

                inj_mv = context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, ref_pair) == EB_FALSE;
#if PRUNE_REF_FRAME_FRO_REC_PARTITION_MVP
                uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
                uint8_t skip_cand = check_ref_beackout(
                    picture_control_set_ptr,
                    context_ptr,
                    to_inject_ref_type,
                    context_ptr->blk_geom->shape);

                if (!skip_cand && (inj_mv)) {
#else
                if (inj_mv)
                {
#endif
#if COMP_MODE

                context_ptr->variance_ready = 0 ;
                    for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types ; cur_type++)
                    {
                // If two predictors are very similar, skip wedge compound mode search
                if (context_ptr->variance_ready)
                    if (context_ptr->prediction_mse < 8 || (!have_newmv_in_inter_mode(NEW_NEARESTMV) && context_ptr->prediction_mse  < 64))
                        continue;
#endif
                    candidateArray[canIdx].type = INTER_MODE;
                    candidateArray[canIdx].inter_mode = NEW_NEARESTMV;
                    candidateArray[canIdx].pred_mode = NEW_NEARESTMV;
                    candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
                    candidateArray[canIdx].is_compound = 1;
#if II_COMP
                    candidateArray[canIdx].is_interintra_used = 0;
#endif
                    candidateArray[canIdx].distortion_ready = 0;
                    candidateArray[canIdx].use_intrabc = 0;

                    candidateArray[canIdx].merge_flag = EB_FALSE;

                    candidateArray[canIdx].prediction_direction[0] = BI_PRED;
                    candidateArray[canIdx].is_new_mv = 0;
                    candidateArray[canIdx].is_zero_mv = 0;
                    candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x_l0;
                    candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y_l0;
                    candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x_l1;
                    candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y_l1;
                    candidateArray[canIdx].drl_index = 0;
                    candidateArray[canIdx].ref_mv_index = 0;
                    candidateArray[canIdx].pred_mv_weight = 0;
                    candidateArray[canIdx].ref_frame_type = ref_pair;
                    candidateArray[canIdx].ref_frame_index_l0 = ref_idx_0;
                    candidateArray[canIdx].ref_frame_index_l1 = ref_idx_1;
#if ATB_TX_TYPE_SUPPORT_PER_TU
                    candidateArray[canIdx].transform_type[0] = DCT_DCT;
                    candidateArray[canIdx].transform_type_uv = DCT_DCT;
#else
                    candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                    candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

#if NEW_NEAR_FIX
                    get_av1_mv_pred_drl(
                        context_ptr,
                        context_ptr->cu_ptr,
                        candidateArray[canIdx].ref_frame_type,
                        candidateArray[canIdx].is_compound,
                        NEW_NEARESTMV,
                        0,//not needed drli,
                        nearestmv,
                        nearmv,
                        ref_mv);
                    candidateArray[canIdx].motion_vector_pred_x[REF_LIST_0] = ref_mv[0].as_mv.col;
                    candidateArray[canIdx].motion_vector_pred_y[REF_LIST_0] = ref_mv[0].as_mv.row;
#else
                    IntMv  bestPredmv[2] = { {0}, {0} };

                    ChooseBestAv1MvPred(
                        context_ptr,
                        candidateArray[canIdx].md_rate_estimation_ptr,
                        context_ptr->cu_ptr,
                        candidateArray[canIdx].ref_frame_type,
                        candidateArray[canIdx].is_compound,
                        candidateArray[canIdx].pred_mode,
                        candidateArray[canIdx].motion_vector_xl0,
                        candidateArray[canIdx].motion_vector_yl0,
                        candidateArray[canIdx].motion_vector_xl1,
                        candidateArray[canIdx].motion_vector_yl1,
                        &candidateArray[canIdx].drl_index,
                        bestPredmv);

                    candidateArray[canIdx].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                    candidateArray[canIdx].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
                    candidateArray[canIdx].motion_vector_pred_x[REF_LIST_1] = bestPredmv[1].as_mv.col;
                    candidateArray[canIdx].motion_vector_pred_y[REF_LIST_1] = bestPredmv[1].as_mv.row;
#endif
                    context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                    context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                    context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                    context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                    context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = ref_pair;
                    ++context_ptr->injected_mv_count_bipred;
#if COMP_MODE
                    //NEW_NEAREST
                    determine_compound_mode(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canIdx],
                        cur_type);
#endif
                        INCRMENT_CAND_TOTAL_COUNT(canIdx);

#if COMP_MODE
                    }
#endif

                }
            }
#if NEW_NEAR_FIX
           //NEW_NEARMV
            if (1) {
                maxDrlIndex = GetMaxDrlIndex(xd->ref_mv_count[ref_pair], NEW_NEARMV);

                for (drli = 0; drli < maxDrlIndex; drli++) {

                    get_av1_mv_pred_drl(
                        context_ptr,
                        context_ptr->cu_ptr,
                        ref_pair,
                        1,
                        NEW_NEARMV,
                        drli,
                        nearestmv,
                        nearmv,
                        ref_mv);

                        //NEW_NEARMV
                        const MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[context_ptr->me_sb_addr];

                        int16_t to_inject_mv_x_l0 = me_results->me_mv_array[context_ptr->me_block_offset][ref_idx_0].x_mv << 1;
                        int16_t to_inject_mv_y_l0 = me_results->me_mv_array[context_ptr->me_block_offset][ref_idx_0].y_mv << 1;
                        int16_t to_inject_mv_x_l1 = nearmv[1].as_mv.col;
                        int16_t to_inject_mv_y_l1 = nearmv[1].as_mv.row;

                        inj_mv = context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, ref_pair) == EB_FALSE;
#if PRUNE_REF_FRAME_FRO_REC_PARTITION_MVP
                        uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
                        uint8_t skip_cand = check_ref_beackout(
                            picture_control_set_ptr,
                            context_ptr,
                            to_inject_ref_type,
                            context_ptr->blk_geom->shape);

                        if (!skip_cand && (inj_mv)) {
#else
                        if (inj_mv) {
#endif

#if COMP_MODE

                context_ptr->variance_ready = 0 ;
                    for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types ; cur_type++)
                        {
                // If two predictors are very similar, skip wedge compound mode search
                if (context_ptr->variance_ready)
                    if (context_ptr->prediction_mse < 8 || (!have_newmv_in_inter_mode(NEW_NEARMV) && context_ptr->prediction_mse  < 64))
                        continue;
#endif
                            candidateArray[canIdx].type = INTER_MODE;
                            candidateArray[canIdx].inter_mode = NEW_NEARMV;
                            candidateArray[canIdx].pred_mode = NEW_NEARMV;
                            candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
                            candidateArray[canIdx].is_compound = 1;
#if II_COMP
                            candidateArray[canIdx].is_interintra_used = 0;
#endif
                            candidateArray[canIdx].distortion_ready = 0;
                            candidateArray[canIdx].use_intrabc = 0;
                            candidateArray[canIdx].merge_flag = EB_FALSE;

                            candidateArray[canIdx].prediction_direction[0] = BI_PRED;
                            candidateArray[canIdx].is_new_mv = 0;
                            candidateArray[canIdx].is_zero_mv = 0;
                            candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x_l0;
                            candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y_l0;
                            candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x_l1;
                            candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y_l1;

                            candidateArray[canIdx].drl_index = drli;

                            candidateArray[canIdx].ref_mv_index = 0;
                            candidateArray[canIdx].pred_mv_weight = 0;
                            candidateArray[canIdx].ref_frame_type = ref_pair;
                            candidateArray[canIdx].ref_frame_index_l0 = ref_idx_0;
                            candidateArray[canIdx].ref_frame_index_l1 = ref_idx_1;
#if ATB_TX_TYPE_SUPPORT_PER_TU
                            candidateArray[canIdx].transform_type[0] = DCT_DCT;
                            candidateArray[canIdx].transform_type_uv = DCT_DCT;
#else
                            candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                            candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif
                            candidateArray[canIdx].motion_vector_pred_x[REF_LIST_0] = ref_mv[0].as_mv.col;
                            candidateArray[canIdx].motion_vector_pred_y[REF_LIST_0] = ref_mv[0].as_mv.row;

                            context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                            context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                            context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                            context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                            context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = ref_pair;
                            ++context_ptr->injected_mv_count_bipred;
#if COMP_MODE
                    //NEW_NEARMV
                    determine_compound_mode(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canIdx],
                        cur_type);
#endif
                            INCRMENT_CAND_TOTAL_COUNT(canIdx);
#if COMP_MODE
                    }
#endif

                        }

                }
           }

           //NEAR_NEWMV
          if (1) {
               maxDrlIndex = GetMaxDrlIndex(xd->ref_mv_count[ref_pair], NEAR_NEWMV);

               for (drli = 0; drli < maxDrlIndex; drli++) {


                   get_av1_mv_pred_drl(
                       context_ptr,
                       context_ptr->cu_ptr,
                       ref_pair,
                       1,
                       NEAR_NEWMV,
                       drli,
                       nearestmv,
                       nearmv,
                       ref_mv);


                   //NEAR_NEWMV
                   const MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[context_ptr->me_sb_addr];

                   int16_t to_inject_mv_x_l0 = nearmv[0].as_mv.col;
                   int16_t to_inject_mv_y_l0 = nearmv[0].as_mv.row;
                   int16_t to_inject_mv_x_l1 = me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (get_list_idx(rf[1]) << 2) : (get_list_idx(rf[1]) << 1)) + ref_idx_1].x_mv << 1;//context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[ref_pair][0].comp_mv.as_mv.col;
                   int16_t to_inject_mv_y_l1 = me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (get_list_idx(rf[1]) << 2) : (get_list_idx(rf[1]) << 1)) + ref_idx_1].y_mv << 1;//context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[ref_pair][0].comp_mv.as_mv.row;

                   inj_mv = context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, ref_pair) == EB_FALSE;
#if PRUNE_REF_FRAME_FRO_REC_PARTITION_MVP
                   uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
                   uint8_t skip_cand = check_ref_beackout(
                   picture_control_set_ptr,
                   context_ptr,
                   to_inject_ref_type,
                   context_ptr->blk_geom->shape);

                   if (!skip_cand && (inj_mv)) {
#else
                   if (inj_mv) {
#endif

#if COMP_MODE
                context_ptr->variance_ready = 0 ;
                    for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types ; cur_type++)
                    {
                // If two predictors are very similar, skip wedge compound mode search
                if (context_ptr->variance_ready)
                    if (context_ptr->prediction_mse < 8 || (!have_newmv_in_inter_mode(NEAR_NEWMV) && context_ptr->prediction_mse  < 64))
                        continue;
#endif
                       candidateArray[canIdx].type = INTER_MODE;
                       candidateArray[canIdx].inter_mode = NEAR_NEWMV;
                       candidateArray[canIdx].pred_mode = NEAR_NEWMV;
                       candidateArray[canIdx].motion_mode = SIMPLE_TRANSLATION;
                       candidateArray[canIdx].is_compound = 1;
#if II_COMP
                       candidateArray[canIdx].is_interintra_used = 0;
#endif
                       candidateArray[canIdx].distortion_ready = 0;
                       candidateArray[canIdx].use_intrabc = 0;
                       candidateArray[canIdx].merge_flag = EB_FALSE;

                       candidateArray[canIdx].prediction_direction[0] = BI_PRED;
                       candidateArray[canIdx].is_new_mv = 0;
                       candidateArray[canIdx].is_zero_mv = 0;
                       candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x_l0;
                       candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y_l0;
                       candidateArray[canIdx].motion_vector_xl1 = to_inject_mv_x_l1;
                       candidateArray[canIdx].motion_vector_yl1 = to_inject_mv_y_l1;
                       candidateArray[canIdx].drl_index = drli;
                       candidateArray[canIdx].ref_mv_index = 0;
                       candidateArray[canIdx].pred_mv_weight = 0;
                       candidateArray[canIdx].ref_frame_type = ref_pair;
                       candidateArray[canIdx].ref_frame_index_l0 = ref_idx_0;
                       candidateArray[canIdx].ref_frame_index_l1 = ref_idx_1;
#if ATB_TX_TYPE_SUPPORT_PER_TU
                       candidateArray[canIdx].transform_type[0] = DCT_DCT;
                       candidateArray[canIdx].transform_type_uv = DCT_DCT;
#else
                       candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                       candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

                       candidateArray[canIdx].motion_vector_pred_x[REF_LIST_1] = ref_mv[1].as_mv.col;
                       candidateArray[canIdx].motion_vector_pred_y[REF_LIST_1] = ref_mv[1].as_mv.row;


                       context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                       context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                       context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                       context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                       context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = ref_pair;
                       ++context_ptr->injected_mv_count_bipred;
#if COMP_MODE
                    //NEAR_NEWMV
                    determine_compound_mode(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canIdx],
                        cur_type);
#endif
                       INCRMENT_CAND_TOTAL_COUNT(canIdx);
#if COMP_MODE
                    }
#endif
                   }
               }
           }
#endif

        }
    }

    //update tot Candidate count
    *candTotCnt = canIdx;
}
#endif

void inject_warped_motion_candidates(
    PictureControlSet              *picture_control_set_ptr,
    struct ModeDecisionContext     *context_ptr,
    CodingUnit                     *cu_ptr,
    uint32_t                       *candTotCnt,
    SsMeContext                    *ss_mecontext,
#if MD_INJECTION
    MeLcuResults                   *meResult,
#if !MEMORY_FOOTPRINT_OPT_ME_MV
    uint16_t                        block_index,
#endif
#else
    MeCuResults                    *mePuResult,
#endif
    EbBool                          use_close_loop_me,
    uint32_t                        close_loop_me_index)
{
    uint32_t canIdx = *candTotCnt;
    ModeDecisionCandidate *candidateArray = context_ptr->fast_candidate_array;
    MacroBlockD  *xd = cu_ptr->av1xd;
    uint8_t drli, maxDrlIndex;
    IntMv nearestmv[2], nearmv[2], ref_mv[2];

    //NEAREST_L0
#if  !MRP_DUPLICATION_FIX
    int16_t to_inject_mv_x = context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.col;
    int16_t to_inject_mv_y = context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.row;
    if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
#endif
        candidateArray[canIdx].type = INTER_MODE;
        candidateArray[canIdx].inter_mode = NEARESTMV;
        candidateArray[canIdx].pred_mode = NEARESTMV;
        candidateArray[canIdx].motion_mode = WARPED_CAUSAL;
        candidateArray[canIdx].wm_params.wmtype = AFFINE;
        candidateArray[canIdx].is_compound = 0;
#if II_COMP
        candidateArray[canIdx].is_interintra_used = 0;
#endif
        candidateArray[canIdx].distortion_ready = 0;
        candidateArray[canIdx].use_intrabc = 0;
        candidateArray[canIdx].merge_flag = EB_FALSE;
        candidateArray[canIdx].prediction_direction[0] = UNI_PRED_LIST_0;
        candidateArray[canIdx].is_new_mv = 0;
        candidateArray[canIdx].is_zero_mv = 0;
#if  !MRP_DUPLICATION_FIX
        candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x;
        candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y;
#else
        candidateArray[canIdx].motion_vector_xl0 = context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.col;
        candidateArray[canIdx].motion_vector_yl0 = context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.row;
#endif
        candidateArray[canIdx].drl_index = 0;
        candidateArray[canIdx].ref_mv_index = 0;
        candidateArray[canIdx].pred_mv_weight = 0;
        candidateArray[canIdx].ref_frame_type = LAST_FRAME;
#if MRP_LIST_REF_IDX_TYPE_LT
        candidateArray[canIdx].ref_frame_index_l0 = 0;
        candidateArray[canIdx].ref_frame_index_l1 = -1;
#endif
#if ATB_TX_TYPE_SUPPORT_PER_TU
        candidateArray[canIdx].transform_type[0] = DCT_DCT;
        candidateArray[canIdx].transform_type_uv = DCT_DCT;
#else
        candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
        candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

        Mv mv_0;
        mv_0.x = candidateArray[canIdx].motion_vector_xl0;
        mv_0.y = candidateArray[canIdx].motion_vector_yl0;
        MvUnit mv_unit;
        mv_unit.mv[0] = mv_0;
        candidateArray[canIdx].local_warp_valid = warped_motion_parameters(
            picture_control_set_ptr,
            context_ptr->cu_ptr,
            &mv_unit,
            context_ptr->blk_geom,
            context_ptr->cu_origin_x,
            context_ptr->cu_origin_y,
            candidateArray[canIdx].ref_frame_type,
            &candidateArray[canIdx].wm_params,
            &candidateArray[canIdx].num_proj_ref);

        if (candidateArray[canIdx].local_warp_valid)
#if CHECK_CAND
            INCRMENT_CAND_TOTAL_COUNT(canIdx);
#else
            ++canIdx;
#endif
#if  !MRP_DUPLICATION_FIX
        context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
        context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
        ++context_ptr->injected_mv_count_l0;
    }
#endif

    //NEAR_L0
    maxDrlIndex = GetMaxDrlIndex(xd->ref_mv_count[LAST_FRAME], NEARMV);
    for (drli = 0; drli < maxDrlIndex; drli++) {
        get_av1_mv_pred_drl(
            context_ptr,
            cu_ptr,
            LAST_FRAME,
            0,
            NEARMV,
            drli,
            nearestmv,
            nearmv,
            ref_mv);
#if  !MRP_DUPLICATION_FIX
        int16_t to_inject_mv_x = nearmv[0].as_mv.col;
        int16_t to_inject_mv_y = nearmv[0].as_mv.row;
        if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
#endif
            candidateArray[canIdx].type = INTER_MODE;
            candidateArray[canIdx].inter_mode = NEARMV;
            candidateArray[canIdx].pred_mode = NEARMV;
            candidateArray[canIdx].motion_mode = WARPED_CAUSAL;
            candidateArray[canIdx].wm_params.wmtype = AFFINE;
            candidateArray[canIdx].is_compound = 0;
#if II_COMP
            candidateArray[canIdx].is_interintra_used = 0;
#endif
            candidateArray[canIdx].distortion_ready = 0;
            candidateArray[canIdx].use_intrabc = 0;
            candidateArray[canIdx].merge_flag = EB_FALSE;
            candidateArray[canIdx].prediction_direction[0] = UNI_PRED_LIST_0;
            candidateArray[canIdx].is_new_mv = 0;
            candidateArray[canIdx].is_zero_mv = 0;
#if !MRP_DUPLICATION_FIX
            candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x;
            candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y;
#else
            candidateArray[canIdx].motion_vector_xl0 = nearmv[0].as_mv.col;
            candidateArray[canIdx].motion_vector_yl0 = nearmv[0].as_mv.row;
#endif
            candidateArray[canIdx].drl_index = drli;
            candidateArray[canIdx].ref_mv_index = 0;
            candidateArray[canIdx].pred_mv_weight = 0;
            candidateArray[canIdx].ref_frame_type = LAST_FRAME;
#if MRP_LIST_REF_IDX_TYPE_LT
            candidateArray[canIdx].ref_frame_index_l0 = 0;
            candidateArray[canIdx].ref_frame_index_l1 = -1;
#endif
#if ATB_TX_TYPE_SUPPORT_PER_TU
            candidateArray[canIdx].transform_type[0] = DCT_DCT;
            candidateArray[canIdx].transform_type_uv = DCT_DCT;
#else
            candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
            candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

            Mv mv_0;
            mv_0.x = candidateArray[canIdx].motion_vector_xl0;
            mv_0.y = candidateArray[canIdx].motion_vector_yl0;
            MvUnit mv_unit;
            mv_unit.mv[0] = mv_0;
            candidateArray[canIdx].local_warp_valid = warped_motion_parameters(
                picture_control_set_ptr,
                context_ptr->cu_ptr,
                &mv_unit,
                context_ptr->blk_geom,
                context_ptr->cu_origin_x,
                context_ptr->cu_origin_y,
                candidateArray[canIdx].ref_frame_type,
                &candidateArray[canIdx].wm_params,
                &candidateArray[canIdx].num_proj_ref);

            if (candidateArray[canIdx].local_warp_valid)
#if CHECK_CAND
                INCRMENT_CAND_TOTAL_COUNT(canIdx);
#else
                ++canIdx;
#endif
#if !MRP_DUPLICATION_FIX
            context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
            context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
            ++context_ptr->injected_mv_count_l0;
        }
#endif
    }

    // NEWMV L0
    const MV neighbors[9] = { { 0, 0 },
        { 0, -2 }, { 2, 0 }, { 0, 2 }, { -2, 0 } ,
        { 2, -2 }, { 2, 2 }, { 2, 2 }, { -2, 2 } };

    IntMv  bestPredmv[2] = { {0}, {0} };
#if MD_INJECTION

#if MEMORY_FOOTPRINT_OPT_ME_MV
    uint8_t total_me_cnt = meResult->total_me_candidate_index[context_ptr->me_block_offset];
    const MeCandidate *me_block_results = meResult->me_candidate[context_ptr->me_block_offset];
#else
    uint8_t total_me_cnt = meResult->total_me_candidate_index[block_index];
    const MeCandidate *me_block_results = meResult->me_candidate[block_index];
#endif
    //const MeLcuResults_t *meResults = pictureControlSetPtr->ParentPcsPtr->meResultsPtr[lcuAddr];
    for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index)
    {
        const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
        const uint8_t inter_direction = me_block_results_ptr->direction;
        const uint8_t list0_ref_index = me_block_results_ptr->ref_idx_l0;
        if (inter_direction == 0) {
#endif
    for (int i=0; i<9; i++){
#if !MRP_DUPLICATION_FIX
        // MD_INJECTION
        int16_t to_inject_mv_x = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : mePuResult->x_mv_l0 << 1; // context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.col;
        int16_t to_inject_mv_y = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : mePuResult->y_mv_l0 << 1; // context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.row;
        to_inject_mv_x += neighbors[i].col;
        to_inject_mv_y += neighbors[i].row;
        if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
#endif

        candidateArray[canIdx].type = INTER_MODE;
        candidateArray[canIdx].distortion_ready = 0;
        candidateArray[canIdx].use_intrabc = 0;
        candidateArray[canIdx].merge_flag = EB_FALSE;
        candidateArray[canIdx].prediction_direction[0] = (EbPredDirection)0;
        candidateArray[canIdx].inter_mode = NEWMV;
        candidateArray[canIdx].pred_mode = NEWMV;
        candidateArray[canIdx].motion_mode = WARPED_CAUSAL;
        candidateArray[canIdx].wm_params.wmtype = AFFINE;

        candidateArray[canIdx].is_compound = 0;
#if II_COMP
        candidateArray[canIdx].is_interintra_used = 0;
#endif
        candidateArray[canIdx].is_new_mv = 1;
        candidateArray[canIdx].is_zero_mv = 0;

        candidateArray[canIdx].drl_index = 0;

        // Set the MV to ME result
#if !MRP_DUPLICATION_FIX
        candidateArray[canIdx].motion_vector_xl0 = to_inject_mv_x;
        candidateArray[canIdx].motion_vector_yl0 = to_inject_mv_y;
#else
#if MD_INJECTION
#if MEMORY_FOOTPRINT_OPT_ME_MV
        candidateArray[canIdx].motion_vector_xl0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : meResult->me_mv_array[context_ptr->me_block_offset][list0_ref_index].x_mv << 1;
        candidateArray[canIdx].motion_vector_yl0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : meResult->me_mv_array[context_ptr->me_block_offset][list0_ref_index].y_mv << 1;
#else
        candidateArray[canIdx].motion_vector_xl0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_block_results_ptr->x_mv_l0 << 1; // context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.col;
        candidateArray[canIdx].motion_vector_yl0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_block_results_ptr->y_mv_l0 << 1; // context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.row;
#endif
#else
        candidateArray[canIdx].motionVector_x_L0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : mePuResult->xMvL0 << 1; // context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.col;
        candidateArray[canIdx].motionVector_y_L0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : mePuResult->yMvL0 << 1; // context_ptr->cu_ptr->ref_mvs[LAST_FRAME][0].as_mv.row;
#endif
        candidateArray[canIdx].motion_vector_xl0 += neighbors[i].col;
        candidateArray[canIdx].motion_vector_yl0 += neighbors[i].row;
#endif
        candidateArray[canIdx].ref_mv_index = 0;
        candidateArray[canIdx].pred_mv_weight = 0;
#if MRP_LIST_REF_IDX_TYPE_LT
        candidateArray[canIdx].ref_frame_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
        candidateArray[canIdx].ref_frame_index_l0 = list0_ref_index;
        candidateArray[canIdx].ref_frame_index_l1 = -1;
#else
        candidateArray[canIdx].ref_frame_type = LAST_FRAME;
#endif

#if ATB_TX_TYPE_SUPPORT_PER_TU
        candidateArray[canIdx].transform_type[0] = DCT_DCT;
        candidateArray[canIdx].transform_type_uv = DCT_DCT;
#else
        candidateArray[canIdx].transform_type[PLANE_TYPE_Y] = DCT_DCT;
        candidateArray[canIdx].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

        ChooseBestAv1MvPred(
            context_ptr,
            candidateArray[canIdx].md_rate_estimation_ptr,
            context_ptr->cu_ptr,
            candidateArray[canIdx].ref_frame_type,
            candidateArray[canIdx].is_compound,
            candidateArray[canIdx].pred_mode,
            candidateArray[canIdx].motion_vector_xl0,
            candidateArray[canIdx].motion_vector_yl0,
            0, 0,
            &candidateArray[canIdx].drl_index,
            bestPredmv);

        candidateArray[canIdx].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
        candidateArray[canIdx].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;

        Mv mv_0;
        mv_0.x = candidateArray[canIdx].motion_vector_xl0;
        mv_0.y = candidateArray[canIdx].motion_vector_yl0;
        MvUnit mv_unit;
        mv_unit.mv[0] = mv_0;
        candidateArray[canIdx].local_warp_valid = warped_motion_parameters(
            picture_control_set_ptr,
            context_ptr->cu_ptr,
            &mv_unit,
            context_ptr->blk_geom,
            context_ptr->cu_origin_x,
            context_ptr->cu_origin_y,
            candidateArray[canIdx].ref_frame_type,
            &candidateArray[canIdx].wm_params,
            &candidateArray[canIdx].num_proj_ref);

        if (candidateArray[canIdx].local_warp_valid)
#if CHECK_CAND
            INCRMENT_CAND_TOTAL_COUNT(canIdx);
#else
            ++canIdx;
#endif
#if !MRP_DUPLICATION_FIX
        context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
        context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
        ++context_ptr->injected_mv_count_l0;
    }
#endif
    }
#if MD_INJECTION
        }
    }
#endif

    *candTotCnt = canIdx;
}




#if ENHANCED_Nx4_4xN_NEW_MV
void inject_new_candidates(
    const SequenceControlSet   *sequence_control_set_ptr,
    struct ModeDecisionContext *context_ptr,
    PictureControlSet          *picture_control_set_ptr,
    EbBool                      isCompoundEnabled,
    EbBool                      allow_bipred,
    uint32_t                    me_sb_addr,
    SsMeContext                *inloop_me_context,
    EbBool                      use_close_loop_me,
    uint32_t                    close_loop_me_index,
    uint32_t                    me_block_offset,
    uint32_t                   *candidateTotalCnt) {

    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
    IntMv  bestPredmv[2] = { {0}, {0} };
    uint32_t canTotalCnt = (*candidateTotalCnt);

    const MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr];
    uint8_t total_me_cnt = me_results->total_me_candidate_index[me_block_offset];
    const MeCandidate *me_block_results = me_results->me_candidate[me_block_offset];
#if COMP_MODE

    BlockSize bsize = context_ptr->blk_geom->bsize;                       // bloc size
    MD_COMP_TYPE compound_types_to_try = picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;
    MD_COMP_TYPE cur_type; //BIP 3x3 MiSize >= BLOCK_8X8 && MiSize <= BLOCK_32X32)
#if COMP_OPT
    //MD_COMP_TYPE tot_comp_types = MD_COMP_AVG;
    MD_COMP_TYPE tot_comp_types = picture_control_set_ptr->parent_pcs_ptr->compound_mode == 1 ? MD_COMP_AVG :
        (bsize >= BLOCK_8X8 && bsize <= BLOCK_32X32) ? compound_types_to_try :
        (compound_types_to_try == MD_COMP_WEDGE) ? MD_COMP_DIFF0 :
        picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;

#if N0_COMP
    tot_comp_types = picture_control_set_ptr->enc_mode == ENC_M0 ? MD_COMP_AVG : tot_comp_types;
#endif

#else
    MD_COMP_TYPE tot_comp_types = (bsize >= BLOCK_8X8 && bsize<= BLOCK_32X32 ) ? compound_types_to_try :
                                  (compound_types_to_try == MD_COMP_WEDGE )? MD_COMP_DIFF0 :
                                   picture_control_set_ptr->parent_pcs_ptr->sequence_control_set_ptr->compound_types_to_try;//MD_COMP_DIST;// MD_COMP_AVG;//
#endif
#endif

    for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index)
    {
        const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
        const uint8_t inter_direction = me_block_results_ptr->direction;
        const uint8_t list0_ref_index = me_block_results_ptr->ref_idx_l0;
        const uint8_t list1_ref_index = me_block_results_ptr->ref_idx_l1;

        /**************
            NEWMV L0
        ************* */
        if (inter_direction == 0) {

#if MEMORY_FOOTPRINT_OPT_ME_MV
            int16_t to_inject_mv_x = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[me_block_offset][list0_ref_index].x_mv << 1;
            int16_t to_inject_mv_y = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[me_block_offset][list0_ref_index].y_mv << 1;
#else
            int16_t to_inject_mv_x = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_block_results_ptr->x_mv_l0 << 1;
            int16_t to_inject_mv_y = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_block_results_ptr->y_mv_l0 << 1;
#endif
#if MRP_DUPLICATION_FIX
            uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
#if PRUNE_REF_FRAME_FRO_REC_PARTITION
            uint8_t skip_cand = check_ref_beackout(
                picture_control_set_ptr,
                context_ptr,
                to_inject_ref_type,
                context_ptr->blk_geom->shape);

            if (!skip_cand && (context_ptr->injected_mv_count_l0 == 0 || mrp_is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE)) {
#else
            if (context_ptr->injected_mv_count_l0 == 0 || mrp_is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE) {
#endif
#else
            if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
#endif


#if II_SEARCH    // NEWMV L0
             MvReferenceFrame rf[2];
             rf[0] = to_inject_ref_type;
             rf[1] = -1;
            uint8_t tot_ii_types =   svt_is_interintra_allowed(picture_control_set_ptr->parent_pcs_ptr->enable_inter_intra,bsize, NEWMV, rf) ? II_COUNT : 1;
            uint8_t ii_type;
            for (ii_type = 0; ii_type < tot_ii_types; ii_type++)
            {
#endif

                candidateArray[canTotalCnt].type = INTER_MODE;
                candidateArray[canTotalCnt].distortion_ready = 0;
                candidateArray[canTotalCnt].use_intrabc = 0;
                candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)0;
                candidateArray[canTotalCnt].inter_mode = NEWMV;
                candidateArray[canTotalCnt].pred_mode = NEWMV;
                candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;

                candidateArray[canTotalCnt].is_compound = 0;
                candidateArray[canTotalCnt].is_new_mv = 1;
                candidateArray[canTotalCnt].is_zero_mv = 0;

                candidateArray[canTotalCnt].drl_index = 0;

                // Set the MV to ME result
                candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x;
                candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y;

                // will be needed later by the rate estimation
                candidateArray[canTotalCnt].ref_mv_index = 0;
                candidateArray[canTotalCnt].pred_mv_weight = 0;
#if MRP_LIST_REF_IDX_TYPE_LT
                candidateArray[canTotalCnt].ref_frame_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                candidateArray[canTotalCnt].ref_frame_index_l0 = list0_ref_index;
                candidateArray[canTotalCnt].ref_frame_index_l1 = -1;
#else
                candidateArray[canTotalCnt].ref_frame_type = LAST_FRAME;
#endif

#if ATB_TX_TYPE_SUPPORT_PER_TU
                candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

                ChooseBestAv1MvPred(
                    context_ptr,
                    candidateArray[canTotalCnt].md_rate_estimation_ptr,
                    context_ptr->cu_ptr,
                    candidateArray[canTotalCnt].ref_frame_type,
                    candidateArray[canTotalCnt].is_compound,
                    candidateArray[canTotalCnt].pred_mode,
                    candidateArray[canTotalCnt].motion_vector_xl0,
                    candidateArray[canTotalCnt].motion_vector_yl0,
                    0, 0,
                    &candidateArray[canTotalCnt].drl_index,
                    bestPredmv);

                candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;

#if II_SEARCH
                candidateArray[canTotalCnt].is_interintra_used = ii_type == 0 ? 0 : 1;
                if (ii_type == 1) {
                    inter_intra_search(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canTotalCnt]);

                    candidateArray[canTotalCnt].use_wedge_interintra = 1;
                    candidateArray[canTotalCnt].ii_wedge_sign = 0;

                }
                if (ii_type == 2)// smooth
                {
                    candidateArray[canTotalCnt].interintra_mode = candidateArray[canTotalCnt-1].interintra_mode;
                    candidateArray[canTotalCnt].use_wedge_interintra = 0;
                }
#endif


#if CHECK_CAND
                INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
                ++canTotalCnt;
#endif
#if II_SEARCH
            }
#endif

                context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
                context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
#if MRP_DUPLICATION_FIX
                context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] = to_inject_ref_type;
#endif
                ++context_ptr->injected_mv_count_l0;
            }

            }

        if (isCompoundEnabled) {
            /**************
               NEWMV L1
           ************* */
            if (inter_direction == 1) {
#if MEMORY_FOOTPRINT_OPT_ME_MV
#if FROM_7_TO_4_MV
                int16_t to_inject_mv_x = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2) + list1_ref_index].x_mv << 1;
                int16_t to_inject_mv_y = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2) + list1_ref_index].y_mv << 1;
#else
                int16_t to_inject_mv_x = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[me_block_offset][4 + list1_ref_index].x_mv << 1;
                int16_t to_inject_mv_y = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[me_block_offset][4 + list1_ref_index].y_mv << 1;
#endif
#else
                int16_t to_inject_mv_x = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_block_results_ptr->x_mv_l1 << 1;
                int16_t to_inject_mv_y = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_block_results_ptr->y_mv_l1 << 1;
#endif
#if MRP_DUPLICATION_FIX
                uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#if PRUNE_REF_FRAME_FRO_REC_PARTITION
            uint8_t skip_cand = check_ref_beackout(
                picture_control_set_ptr,
                context_ptr,
                to_inject_ref_type,
                context_ptr->blk_geom->shape);

                if (!skip_cand && (context_ptr->injected_mv_count_l1 == 0 || mrp_is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE)) {
#else
                if (context_ptr->injected_mv_count_l1 == 0 || mrp_is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE) {
#endif
#else
                if (context_ptr->injected_mv_count_l1 == 0 || is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
#endif

#if II_SEARCH // NEWMV L1
             MvReferenceFrame rf[2];
             rf[0] = to_inject_ref_type;
             rf[1] = -1;
            uint8_t tot_ii_types =    svt_is_interintra_allowed(picture_control_set_ptr->parent_pcs_ptr->enable_inter_intra,bsize, NEWMV, rf) ? II_COUNT : 1;
            uint8_t ii_type;
            for (ii_type = 0; ii_type < tot_ii_types; ii_type++)
            {
#endif
                    candidateArray[canTotalCnt].type = INTER_MODE;
                    candidateArray[canTotalCnt].distortion_ready = 0;
                    candidateArray[canTotalCnt].use_intrabc = 0;
                    candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                    candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)1;

                    candidateArray[canTotalCnt].inter_mode = NEWMV;
                    candidateArray[canTotalCnt].pred_mode = NEWMV;
                    candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;

                    candidateArray[canTotalCnt].is_compound = 0;
                    candidateArray[canTotalCnt].is_new_mv = 1;
                    candidateArray[canTotalCnt].is_zero_mv = 0;

                    candidateArray[canTotalCnt].drl_index = 0;

                    // Set the MV to ME result
                    candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x;
                    candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y;

                    // will be needed later by the rate estimation
                    candidateArray[canTotalCnt].ref_mv_index = 0;
                    candidateArray[canTotalCnt].pred_mv_weight = 0;
#if MRP_LIST_REF_IDX_TYPE_LT
                    candidateArray[canTotalCnt].ref_frame_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
                    candidateArray[canTotalCnt].ref_frame_index_l0 = -1;
                    candidateArray[canTotalCnt].ref_frame_index_l1 = list1_ref_index;
#else
                    candidateArray[canTotalCnt].ref_frame_type = BWDREF_FRAME;
#endif

#if ATB_TX_TYPE_SUPPORT_PER_TU
                    candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                    candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
                    candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                    candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

                    ChooseBestAv1MvPred(
                        context_ptr,
                        candidateArray[canTotalCnt].md_rate_estimation_ptr,
                        context_ptr->cu_ptr,
                        candidateArray[canTotalCnt].ref_frame_type,
                        candidateArray[canTotalCnt].is_compound,
                        candidateArray[canTotalCnt].pred_mode,
                        candidateArray[canTotalCnt].motion_vector_xl1,
                        candidateArray[canTotalCnt].motion_vector_yl1,
                        0, 0,
                        &candidateArray[canTotalCnt].drl_index,
                        bestPredmv);

                    candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[0].as_mv.col;
                    candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[0].as_mv.row;
#if II_SEARCH
                candidateArray[canTotalCnt].is_interintra_used = ii_type == 0 ? 0 : 1;
                if (ii_type == 1) {
                    inter_intra_search(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canTotalCnt]);

                    candidateArray[canTotalCnt].use_wedge_interintra = 1;
                    candidateArray[canTotalCnt].ii_wedge_sign = 0;

                }
                if (ii_type == 2)// smooth
                {
                    candidateArray[canTotalCnt].interintra_mode = candidateArray[canTotalCnt-1].interintra_mode;
                    candidateArray[canTotalCnt].use_wedge_interintra = 0;
                }
#endif


#if CHECK_CAND
                    INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
                    ++canTotalCnt;
#endif

#if II_SEARCH
            }
#endif
                    context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_x;
                    context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_y;
#if MRP_DUPLICATION_FIX
                    context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] = to_inject_ref_type;
#endif
                    ++context_ptr->injected_mv_count_l1;
                }

                }
            /**************
               NEW_NEWMV
            ************* */
            if (allow_bipred) {

                if (inter_direction == 2) {
#if MEMORY_FOOTPRINT_OPT_ME_MV
                    int16_t to_inject_mv_x_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[me_block_offset][list0_ref_index].x_mv << 1;
                    int16_t to_inject_mv_y_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[me_block_offset][list0_ref_index].y_mv << 1;
#if FROM_7_TO_4_MV
                    int16_t to_inject_mv_x_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].x_mv << 1;
                    int16_t to_inject_mv_y_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].y_mv << 1;
#else
                    int16_t to_inject_mv_x_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[me_block_offset][(me_block_results_ptr->ref1_list << 2) + list1_ref_index].x_mv << 1;
                    int16_t to_inject_mv_y_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[me_block_offset][(me_block_results_ptr->ref1_list << 2) + list1_ref_index].y_mv << 1;
#endif
#else
                    int16_t to_inject_mv_x_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_block_results_ptr->x_mv_l0 << 1;
                    int16_t to_inject_mv_y_l0 = use_close_loop_me ? inloop_me_context->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_block_results_ptr->y_mv_l0 << 1;
                    int16_t to_inject_mv_x_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_block_results_ptr->x_mv_l1 << 1;
                    int16_t to_inject_mv_y_l1 = use_close_loop_me ? inloop_me_context->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_block_results_ptr->y_mv_l1 << 1;
#endif
#if MRP_DUPLICATION_FIX
                    MvReferenceFrame rf[2];
#if MRP_MD_UNI_DIR_BIPRED
                    rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
                    rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
#else
                    rf[0] = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                    rf[1] = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#endif
                    uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
#if PRUNE_REF_FRAME_FRO_REC_PARTITION
                    uint8_t skip_cand = check_ref_beackout(
                        picture_control_set_ptr,
                        context_ptr,
                        to_inject_ref_type,
                        context_ptr->blk_geom->shape);

                    if (!skip_cand && (context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, to_inject_ref_type) == EB_FALSE)) {
#else
                    if (context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, to_inject_ref_type) == EB_FALSE) {
#endif
#else
                    if (context_ptr->injected_mv_count_bipred == 0 || is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1) == EB_FALSE) {
#endif

#if COMP_MODE
                context_ptr->variance_ready = 0 ;
                        for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types ; cur_type++)
                        {
                // If two predictors are very similar, skip wedge compound mode search
                if (context_ptr->variance_ready)
                    if (context_ptr->prediction_mse < 8 || (!have_newmv_in_inter_mode(NEW_NEWMV) && context_ptr->prediction_mse  < 64))
                        continue;

#endif
                            candidateArray[canTotalCnt].type = INTER_MODE;

                        candidateArray[canTotalCnt].distortion_ready = 0;
                        candidateArray[canTotalCnt].use_intrabc = 0;

                        candidateArray[canTotalCnt].merge_flag = EB_FALSE;

                        candidateArray[canTotalCnt].is_new_mv = 1;
                        candidateArray[canTotalCnt].is_zero_mv = 0;

                        candidateArray[canTotalCnt].drl_index = 0;

                        // Set the MV to ME result

                        candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x_l0;
                        candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y_l0;
                        candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x_l1;
                        candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y_l1;

                        // will be needed later by the rate estimation
                        candidateArray[canTotalCnt].ref_mv_index = 0;
                        candidateArray[canTotalCnt].pred_mv_weight = 0;

                        candidateArray[canTotalCnt].inter_mode = NEW_NEWMV;
                        candidateArray[canTotalCnt].pred_mode = NEW_NEWMV;
                        candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                        candidateArray[canTotalCnt].is_compound = 1;
#if II_COMP
                        candidateArray[canTotalCnt].is_interintra_used = 0;
#endif
                        candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)2;
#if MRP_LIST_REF_IDX_TYPE_LT
                        MvReferenceFrame rf[2];
#if MRP_MD_UNI_DIR_BIPRED
                        rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
                        rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
#else
                        rf[0] = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                        rf[1] = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#endif
                        candidateArray[canTotalCnt].ref_frame_type = av1_ref_frame_type(rf);

                        candidateArray[canTotalCnt].ref_frame_index_l0 = list0_ref_index;
                        candidateArray[canTotalCnt].ref_frame_index_l1 = list1_ref_index;
#else
                        candidateArray[canTotalCnt].ref_frame_type = LAST_BWD_FRAME;
#endif
#if ATB_TX_TYPE_SUPPORT_PER_TU
                        candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                        candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
                        candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                        candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif
                        ChooseBestAv1MvPred(
                            context_ptr,
                            candidateArray[canTotalCnt].md_rate_estimation_ptr,
                            context_ptr->cu_ptr,
                            candidateArray[canTotalCnt].ref_frame_type,
                            candidateArray[canTotalCnt].is_compound,
                            candidateArray[canTotalCnt].pred_mode,
                            candidateArray[canTotalCnt].motion_vector_xl0,
                            candidateArray[canTotalCnt].motion_vector_yl0,
                            candidateArray[canTotalCnt].motion_vector_xl1,
                            candidateArray[canTotalCnt].motion_vector_yl1,
                            &candidateArray[canTotalCnt].drl_index,
                            bestPredmv);

                        candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                        candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
                        candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[1].as_mv.col;
                        candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[1].as_mv.row;
#if COMP_MODE
                            //NEW_NEW
                            determine_compound_mode(
                                picture_control_set_ptr,
                                context_ptr,
                                &candidateArray[canTotalCnt],
                                cur_type);
#endif
#if CHECK_CAND
                        INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
                        ++canTotalCnt;
#endif

                        context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                        context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                        context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                        context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
#if MRP_DUPLICATION_FIX
                        context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
#endif
                        ++context_ptr->injected_mv_count_bipred;
#if COMP_MODE
                        }
#endif
                    }

                    }
                }
            }
            }
    // update the total number of candidates injected
    (*candidateTotalCnt) = canTotalCnt;
        }
#endif
#if COMP_MODE
extern aom_variance_fn_ptr_t mefn_ptr[BlockSizeS_ALL];
#endif

#if PREDICTIVE_ME // inject them
void inject_predictive_me_candidates(
    //const SequenceControlSet   *sequence_control_set_ptr,
    struct ModeDecisionContext *context_ptr,
    PictureControlSet          *picture_control_set_ptr,
    EbBool                      isCompoundEnabled,
    EbBool                      allow_bipred,
    uint32_t                   *candidateTotalCnt) {

    ModeDecisionCandidate *candidateArray = context_ptr->fast_candidate_array;
    IntMv  bestPredmv[2] = { {0}, {0} };
    uint32_t canTotalCnt = (*candidateTotalCnt);

#if COMP_MODE
    BlockSize bsize = context_ptr->blk_geom->bsize;                       // bloc size
    MD_COMP_TYPE compound_types_to_try = picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;
    MD_COMP_TYPE cur_type; //BIP 3x3 MiSize >= BLOCK_8X8 && MiSize <= BLOCK_32X32)
    MD_COMP_TYPE tot_comp_types = (bsize >= BLOCK_8X8 && bsize <= BLOCK_32X32) ? compound_types_to_try :
        (compound_types_to_try == MD_COMP_WEDGE) ? MD_COMP_DIFF0 :
        picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;//MD_COMP_DIST;// MD_COMP_AVG;//
#if 0//N0_COMP
    tot_comp_types = picture_control_set_ptr->enc_mode == ENC_M0 ? MD_COMP_AVG : tot_comp_types;
#endif

#endif

    uint8_t listIndex;
    uint8_t ref_pic_index;
    listIndex = REF_LIST_0;
    {
        // Ref Picture Loop
        for (ref_pic_index = 0; ref_pic_index < 4; ++ref_pic_index) {
            if (context_ptr->valid_refined_mv[listIndex][ref_pic_index]) {
                int16_t to_inject_mv_x = context_ptr->best_spatial_pred_mv[listIndex][ref_pic_index][0];
                int16_t to_inject_mv_y = context_ptr->best_spatial_pred_mv[listIndex][ref_pic_index][1];
                uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0, ref_pic_index);
                if (context_ptr->injected_mv_count_l0 == 0 || mrp_is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE) {

                    candidateArray[canTotalCnt].type = INTER_MODE;
                    candidateArray[canTotalCnt].distortion_ready = 0;
                    candidateArray[canTotalCnt].use_intrabc = 0;
                    candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                    candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)0;
                    candidateArray[canTotalCnt].inter_mode = NEWMV;
                    candidateArray[canTotalCnt].pred_mode = NEWMV;
                    candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                    candidateArray[canTotalCnt].is_compound = 0;
#if II_SEARCH // PME OFF   L0
                    candidateArray[canTotalCnt].is_interintra_used = 0;
#endif
                    candidateArray[canTotalCnt].is_new_mv = 1;
                    candidateArray[canTotalCnt].is_zero_mv = 0;
                    candidateArray[canTotalCnt].drl_index = 0;
                    candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x;
                    candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y;
                    candidateArray[canTotalCnt].ref_mv_index = 0;
                    candidateArray[canTotalCnt].pred_mv_weight = 0;
                    candidateArray[canTotalCnt].ref_frame_type = svt_get_ref_frame_type(REF_LIST_0, ref_pic_index);
                    candidateArray[canTotalCnt].ref_frame_index_l0 = ref_pic_index;
                    candidateArray[canTotalCnt].ref_frame_index_l1 = -1;
                    candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                    candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;

                    ChooseBestAv1MvPred(
                        context_ptr,
                        candidateArray[canTotalCnt].md_rate_estimation_ptr,
                        context_ptr->cu_ptr,
                        candidateArray[canTotalCnt].ref_frame_type,
                        candidateArray[canTotalCnt].is_compound,
                        candidateArray[canTotalCnt].pred_mode,
                        candidateArray[canTotalCnt].motion_vector_xl0,
                        candidateArray[canTotalCnt].motion_vector_yl0,
                        0, 0,
                        &candidateArray[canTotalCnt].drl_index,
                        bestPredmv);

                    candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                    candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
                    INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
                    context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
                    context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
                    context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] = to_inject_ref_type;
                    ++context_ptr->injected_mv_count_l0;
                }
            }
        }
    }
    if (isCompoundEnabled) {
        /**************
           NEWMV L1
       ************* */
        listIndex = REF_LIST_1;
        {
            // Ref Picture Loop
            for (ref_pic_index = 0; ref_pic_index < 3; ++ref_pic_index) {
                if (context_ptr->valid_refined_mv[listIndex][ref_pic_index]) {
                    int16_t to_inject_mv_x = context_ptr->best_spatial_pred_mv[listIndex][ref_pic_index][0];
                    int16_t to_inject_mv_y = context_ptr->best_spatial_pred_mv[listIndex][ref_pic_index][1];
                    uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_1, ref_pic_index);
                    if (context_ptr->injected_mv_count_l1 == 0 || mrp_is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE) {

                        candidateArray[canTotalCnt].type = INTER_MODE;
                        candidateArray[canTotalCnt].distortion_ready = 0;
                        candidateArray[canTotalCnt].use_intrabc = 0;
                        candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                        candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)1;
                        candidateArray[canTotalCnt].inter_mode = NEWMV;
                        candidateArray[canTotalCnt].pred_mode = NEWMV;
                        candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                        candidateArray[canTotalCnt].is_compound = 0;
#if II_SEARCH // PME OFF   L1
                        candidateArray[canTotalCnt].is_interintra_used = 0;
#endif
                        candidateArray[canTotalCnt].is_new_mv = 1;
                        candidateArray[canTotalCnt].is_zero_mv = 0;
                        candidateArray[canTotalCnt].drl_index = 0;
                        candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x;
                        candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y;
                        candidateArray[canTotalCnt].ref_mv_index = 0;
                        candidateArray[canTotalCnt].pred_mv_weight = 0;
                        candidateArray[canTotalCnt].ref_frame_type = svt_get_ref_frame_type(REF_LIST_1, ref_pic_index);
                        candidateArray[canTotalCnt].ref_frame_index_l0 = -1;
                        candidateArray[canTotalCnt].ref_frame_index_l1 = ref_pic_index;
                        candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                        candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;

                        ChooseBestAv1MvPred(
                            context_ptr,
                            candidateArray[canTotalCnt].md_rate_estimation_ptr,
                            context_ptr->cu_ptr,
                            candidateArray[canTotalCnt].ref_frame_type,
                            candidateArray[canTotalCnt].is_compound,
                            candidateArray[canTotalCnt].pred_mode,
                            candidateArray[canTotalCnt].motion_vector_xl1,
                            candidateArray[canTotalCnt].motion_vector_yl1,
                            0, 0,
                            &candidateArray[canTotalCnt].drl_index,
                            bestPredmv);

                        candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[0].as_mv.col;
                        candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[0].as_mv.row;
                        INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
                        context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_x;
                        context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_y;
                        context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] = to_inject_ref_type;
                        ++context_ptr->injected_mv_count_l1;
                    }
                }
            }
        }
    }
    /**************
        NEW_NEWMV
    ************* */
    if (allow_bipred) {
        uint8_t ref_pic_index_l0;
        uint8_t ref_pic_index_l1;
        {
            // Ref Picture Loop
            for (ref_pic_index_l0 = 0; ref_pic_index_l0 < 4; ++ref_pic_index_l0) {
                for (ref_pic_index_l1 = 0; ref_pic_index_l1 < 4; ++ref_pic_index_l1) {
                    if (context_ptr->valid_refined_mv[REF_LIST_0][ref_pic_index_l0] && context_ptr->valid_refined_mv[REF_LIST_1][ref_pic_index_l1]) {
                        int16_t to_inject_mv_x_l0 = context_ptr->best_spatial_pred_mv[REF_LIST_0][ref_pic_index_l0][0];
                        int16_t to_inject_mv_y_l0 = context_ptr->best_spatial_pred_mv[REF_LIST_0][ref_pic_index_l0][1];
                        int16_t to_inject_mv_x_l1 = context_ptr->best_spatial_pred_mv[REF_LIST_1][ref_pic_index_l1][0];
                        int16_t to_inject_mv_y_l1 = context_ptr->best_spatial_pred_mv[REF_LIST_1][ref_pic_index_l1][1];

                        MvReferenceFrame rf[2];
                        rf[0] = svt_get_ref_frame_type(REF_LIST_0, ref_pic_index_l0);
                        rf[1] = svt_get_ref_frame_type(REF_LIST_1, ref_pic_index_l1);
                        uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
                        if (context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, to_inject_ref_type) == EB_FALSE) {

#if COMP_MODE
                            context_ptr->variance_ready = 0;
                            for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types; cur_type++)
                            {
                                // If two predictors are very similar, skip wedge compound mode search
                                if (context_ptr->variance_ready)
                                    if (context_ptr->prediction_mse < 8 || (!have_newmv_in_inter_mode(NEW_NEWMV) && context_ptr->prediction_mse < 64))
                                        continue;

#endif
                                candidateArray[canTotalCnt].type = INTER_MODE;
                                candidateArray[canTotalCnt].distortion_ready = 0;
                                candidateArray[canTotalCnt].use_intrabc = 0;
                                candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                                candidateArray[canTotalCnt].is_new_mv = 1;
                                candidateArray[canTotalCnt].is_zero_mv = 0;
                                candidateArray[canTotalCnt].drl_index = 0;
                                // Set the MV to ME result
                                candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x_l0;
                                candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y_l0;
                                candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x_l1;
                                candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y_l1;
                                // will be needed later by the rate estimation
                                candidateArray[canTotalCnt].ref_mv_index = 0;
                                candidateArray[canTotalCnt].pred_mv_weight = 0;
                                candidateArray[canTotalCnt].inter_mode = NEW_NEWMV;
                                candidateArray[canTotalCnt].pred_mode = NEW_NEWMV;
                                candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                                candidateArray[canTotalCnt].is_compound = 1;
#if II_COMP
                                candidateArray[canTotalCnt].is_interintra_used = 0;
#endif
                                candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)2;

                                MvReferenceFrame rf[2];
                                rf[0] = svt_get_ref_frame_type(REF_LIST_0, ref_pic_index_l0);
                                rf[1] = svt_get_ref_frame_type(REF_LIST_1, ref_pic_index_l1);
                                candidateArray[canTotalCnt].ref_frame_type = av1_ref_frame_type(rf);
                                candidateArray[canTotalCnt].ref_frame_index_l0 = ref_pic_index_l0;
                                candidateArray[canTotalCnt].ref_frame_index_l1 = ref_pic_index_l1;

                                candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                                candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;

                                ChooseBestAv1MvPred(
                                    context_ptr,
                                    candidateArray[canTotalCnt].md_rate_estimation_ptr,
                                    context_ptr->cu_ptr,
                                    candidateArray[canTotalCnt].ref_frame_type,
                                    candidateArray[canTotalCnt].is_compound,
                                    candidateArray[canTotalCnt].pred_mode,
                                    candidateArray[canTotalCnt].motion_vector_xl0,
                                    candidateArray[canTotalCnt].motion_vector_yl0,
                                    candidateArray[canTotalCnt].motion_vector_xl1,
                                    candidateArray[canTotalCnt].motion_vector_yl1,
                                    &candidateArray[canTotalCnt].drl_index,
                                    bestPredmv);
                                candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                                candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
                                candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[1].as_mv.col;
                                candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[1].as_mv.row;

#if COMP_MODE
                                //MVP REFINE
                                determine_compound_mode(
                                    picture_control_set_ptr,
                                    context_ptr,
                                    &candidateArray[canTotalCnt],
                                    cur_type);
#endif
                                INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
                                context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                                context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                                context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                                context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;

                                context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
                                ++context_ptr->injected_mv_count_bipred;
                            }
#if COMP_MODE
                        }
#endif
                    }
                }
            }
        }
    }

    (*candidateTotalCnt) = canTotalCnt;
}
#endif


#if MD_INJECTION
void  inject_inter_candidates(
    PictureControlSet            *picture_control_set_ptr,
    ModeDecisionContext          *context_ptr,
    SsMeContext                  *ss_mecontext,
    const SequenceControlSet     *sequence_control_set_ptr,
    LargestCodingUnit            *sb_ptr,
#if M8_SKIP_BLK
    uint32_t                       *candidateTotalCnt) {
#else
    uint32_t                       *candidateTotalCnt,
    const uint32_t                  leaf_index){
#endif

    (void)sequence_control_set_ptr;
    uint32_t                   canTotalCnt = *candidateTotalCnt;
    //const uint32_t             lcuAddr = sb_ptr->index;
    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
    EbBool isCompoundEnabled = (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
#if !NEW_NEAREST_NEW_INJECTION && !ENHANCED_Nx4_4xN_NEW_MV
    uint32_t me_sb_addr;
#endif
#if !PREDICTIVE_ME
    uint32_t geom_offset_x = 0;
    uint32_t geom_offset_y = 0;

    if (sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128) {
        uint32_t me_sb_size = sequence_control_set_ptr->sb_sz;
        uint32_t me_pic_width_in_sb = (sequence_control_set_ptr->seq_header.max_frame_width + sequence_control_set_ptr->sb_sz - 1) / me_sb_size;
        uint32_t me_sb_x = (context_ptr->cu_origin_x / me_sb_size);
        uint32_t me_sb_y = (context_ptr->cu_origin_y / me_sb_size);
#if NEW_NEAREST_NEW_INJECTION || ENHANCED_Nx4_4xN_NEW_MV
        context_ptr->me_sb_addr = me_sb_x + me_sb_y * me_pic_width_in_sb;
#else
        me_sb_addr = me_sb_x + me_sb_y * me_pic_width_in_sb;
#endif
        geom_offset_x = (me_sb_x & 0x1) * me_sb_size;
        geom_offset_y = (me_sb_y & 0x1) * me_sb_size;
    }
    else
#if NEW_NEAREST_NEW_INJECTION || ENHANCED_Nx4_4xN_NEW_MV
        context_ptr->me_sb_addr = lcuAddr;
#else
        me_sb_addr = lcuAddr;
#endif
#endif
#if !PREDICTIVE_ME
    uint32_t max_number_of_pus_per_sb;

    max_number_of_pus_per_sb = picture_control_set_ptr->parent_pcs_ptr->max_number_of_pus_per_sb;
#endif
#if MEMORY_FOOTPRINT_OPT_ME_MV
#if !PREDICTIVE_ME
    context_ptr->me_block_offset =
        (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4 || context_ptr->blk_geom->bwidth == 128 || context_ptr->blk_geom->bheight == 128) ?
            0 :
            get_me_info_index(max_number_of_pus_per_sb, context_ptr->blk_geom, geom_offset_x, geom_offset_y);
#endif
#else
    uint32_t me2Nx2NTableOffset;

    me2Nx2NTableOffset = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4 || context_ptr->blk_geom->bwidth == 128 || context_ptr->blk_geom->bheight == 128) ? 0 :
        get_me_info_index(max_number_of_pus_per_sb, context_ptr->blk_geom, geom_offset_x, geom_offset_y);
#endif

#if MD_INJECTION
#if NEW_NEAREST_NEW_INJECTION
    MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[context_ptr->me_sb_addr];
#else
    MeLcuResults *me_results            = picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr];
#endif

#if MEMORY_FOOTPRINT_OPT_ME_MV
#if !ENHANCED_Nx4_4xN_NEW_MV
    uint8_t total_me_cnt = me_results->total_me_candidate_index[context_ptr->me_block_offset];
    const MeCandidate *me_block_results = me_results->me_candidate[context_ptr->me_block_offset];
#endif
#else
    uint8_t total_me_cnt = me_results->total_me_candidate_index[me2Nx2NTableOffset];
    const MeCandidate *me_block_results = me_results->me_candidate[me2Nx2NTableOffset];
#endif
#else
    MeCuResults_t * mePuResult = &picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr][me2Nx2NTableOffset];
#endif
    EbBool use_close_loop_me = picture_control_set_ptr->parent_pcs_ptr->enable_in_loop_motion_estimation_flag &&
        ((context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) || (context_ptr->blk_geom->bwidth > 64 || context_ptr->blk_geom->bheight > 64)) ? EB_TRUE : EB_FALSE;

    uint32_t close_loop_me_index = use_close_loop_me ? get_in_loop_me_info_index(MAX_SS_ME_PU_COUNT, sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128 ? 1 : 0, context_ptr->blk_geom) : 0;
#if BASE_LAYER_REF || MRP_REF_MODE
#if MRP_ENABLE_BI_FOR_BASE
    EbBool allow_bipred = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) ? EB_FALSE : EB_TRUE;
#if MRP_DISABLE_ADDED_CAND_M1
    allow_bipred = (picture_control_set_ptr->enc_mode >= ENC_M1 && picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0) ? EB_FALSE : allow_bipred;
#endif
#if !INJ_MVP
   EbBool amp_allow_bipred = (picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0 || context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) ? EB_FALSE : EB_TRUE;
#endif
#else
    EbBool allow_bipred = (picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0 || context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) ? EB_FALSE : EB_TRUE;
#endif
#else
    EbBool allow_bipred = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) ? EB_FALSE : EB_TRUE;
#endif
#if !ENHANCED_Nx4_4xN_NEW_MV
    IntMv  bestPredmv[2] = { {0}, {0} };
#endif
    uint8_t sq_index = LOG2F(context_ptr->blk_geom->sq_size) - 2;
    uint8_t inject_newmv_candidate = 1;

#if ADP_BQ
    // to add the support for extra partitioning method here
#endif

#if COMP_MODE

    BlockSize bsize = context_ptr->blk_geom->bsize;                       // bloc size
    MD_COMP_TYPE compound_types_to_try = picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;
    MD_COMP_TYPE cur_type; //BIP 3x3 MiSize >= BLOCK_8X8 && MiSize <= BLOCK_32X32)
#if COMP_OPT
    //MD_COMP_TYPE tot_comp_types = MD_COMP_AVG;
    MD_COMP_TYPE tot_comp_types = picture_control_set_ptr->parent_pcs_ptr->compound_mode == 1 ? MD_COMP_AVG :
        (bsize >= BLOCK_8X8 && bsize <= BLOCK_32X32) ? compound_types_to_try :
        (compound_types_to_try == MD_COMP_WEDGE) ? MD_COMP_DIFF0 :
        picture_control_set_ptr->parent_pcs_ptr->compound_types_to_try;
#if N0_COMP
    tot_comp_types = picture_control_set_ptr->enc_mode == ENC_M0 ? MD_COMP_AVG : tot_comp_types;
#endif

#else
    MD_COMP_TYPE tot_comp_types = (bsize >= BLOCK_8X8 && bsize<= BLOCK_32X32 ) ? compound_types_to_try :
                                  (compound_types_to_try == MD_COMP_WEDGE )? MD_COMP_DIFF0 :
                                   picture_control_set_ptr->parent_pcs_ptr->sequence_control_set_ptr->compound_types_to_try;//MD_COMP_DIST;// MD_COMP_AVG;//
#endif
#endif
#if ADP_BQ
    if (picture_control_set_ptr->parent_pcs_ptr->pic_depth_mode == PIC_SB_SWITCH_NSQ_DEPTH_MODE || (picture_control_set_ptr->parent_pcs_ptr->nsq_search_level >= NSQ_SEARCH_LEVEL1 && picture_control_set_ptr->parent_pcs_ptr->nsq_search_level < NSQ_SEARCH_FULL)) {
#else
    if (picture_control_set_ptr->parent_pcs_ptr->nsq_search_level >= NSQ_SEARCH_LEVEL1 &&
        picture_control_set_ptr->parent_pcs_ptr->nsq_search_level < NSQ_SEARCH_FULL) {
#endif
        inject_newmv_candidate = context_ptr->blk_geom->shape == PART_N ? 1 :
            context_ptr->parent_sq_has_coeff[sq_index] != 0 ? inject_newmv_candidate : 0;
    }
#if !PREDICTIVE_ME
    generate_av1_mvp_table(
        &sb_ptr->tile_info,
        context_ptr,
        context_ptr->cu_ptr,
        context_ptr->blk_geom,
        context_ptr->cu_origin_x,
        context_ptr->cu_origin_y,
#if MRP_MVP
        picture_control_set_ptr->parent_pcs_ptr->ref_frame_type_arr,
#if RPS_4L
        picture_control_set_ptr->parent_pcs_ptr->tot_ref_frame_types,
#else
        (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) ? 1 : picture_control_set_ptr->parent_pcs_ptr->tot_ref_frame_types,
#endif
#else
        refFrames,
        (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) ? 1 : 3,
#endif
        picture_control_set_ptr);
#endif

    uint32_t mi_row = context_ptr->cu_origin_y >> MI_SIZE_LOG2;
    uint32_t mi_col = context_ptr->cu_origin_x >> MI_SIZE_LOG2;
    av1_count_overlappable_neighbors(
        picture_control_set_ptr,
        context_ptr->cu_ptr,
        context_ptr->blk_geom->bsize,
        mi_row,
        mi_col);

    /**************
         MVP
    ************* */

#if INJ_MVP
    uint32_t refIt;
    //all of ref pairs: (1)single-ref List0  (2)single-ref List1  (3)compound Bi-Dir List0-List1  (4)compound Uni-Dir List0-List0  (5)compound Uni-Dir List1-List1
    for (refIt = 0; refIt < picture_control_set_ptr->parent_pcs_ptr->tot_ref_frame_types; ++refIt) {
        MvReferenceFrame ref_frame_pair = picture_control_set_ptr->parent_pcs_ptr->ref_frame_type_arr[refIt];
        inject_mvp_candidates_II(
            context_ptr,
            picture_control_set_ptr,
            context_ptr->cu_ptr,
            ref_frame_pair,
            &canTotalCnt);
    }
#else

    InjectAv1MvpCandidates(
        context_ptr,
        context_ptr->cu_ptr,
        refFrames,
        picture_control_set_ptr,
        lcuAddr,
#if !M8_SKIP_BLK
        leaf_index,
#endif
        amp_allow_bipred,
        &canTotalCnt);

#endif

#if NEW_NEAREST_NEW_INJECTION
    //----------------------
    //    NEAREST_NEWMV, NEW_NEARESTMV, NEAR_NEWMV, NEW_NEARMV.
    //----------------------
    if (context_ptr->new_nearest_near_comb_injection) {
        EbBool allow_compound = (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE || context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) ? EB_FALSE : EB_TRUE;
        if (allow_compound) {
            //all of ref pairs: (1)single-ref List0  (2)single-ref List1  (3)compound Bi-Dir List0-List1  (4)compound Uni-Dir List0-List0  (5)compound Uni-Dir List1-List1
            for (refIt = 0; refIt < picture_control_set_ptr->parent_pcs_ptr->tot_ref_frame_types; ++refIt) {
                MvReferenceFrame ref_frame_pair = picture_control_set_ptr->parent_pcs_ptr->ref_frame_type_arr[refIt];
                inject_new_nearest_new_comb_candidates(
                    sequence_control_set_ptr,
                    context_ptr,
                    picture_control_set_ptr,
                    ref_frame_pair,
                    &canTotalCnt);
            }
        }
    }
#endif

    if (inject_newmv_candidate) {

#if ENHANCED_Nx4_4xN_NEW_MV
        inject_new_candidates(
            sequence_control_set_ptr,
            context_ptr,
            picture_control_set_ptr,
            isCompoundEnabled,
            allow_bipred,
            context_ptr->me_sb_addr,
            ss_mecontext,
            use_close_loop_me,
            close_loop_me_index,
            context_ptr->me_block_offset,
            &canTotalCnt);

        if (context_ptr->nx4_4xn_parent_mv_injection) {
            // If Nx4 or 4xN the inject the MV of the aprent block


            // Derive whether if current block would need to have offsets made
            uint32_t bwidth_offset_to_8 = (context_ptr->blk_geom->bwidth == 4) << 2;
            uint32_t bheight_offset_to_8 = (context_ptr->blk_geom->bheight == 4) << 2;

            // if there is an offset needed to set either dimension to 8
            if (bwidth_offset_to_8 || bheight_offset_to_8) {

                // Align parent block has dimensions inherited by current block, if current block has a dimension of 4
                // add 4 so the resulting block follows an 8x8 basis
                uint32_t bwidth_to_search = context_ptr->blk_geom->bwidth + bwidth_offset_to_8;
                uint32_t bheight_to_search = context_ptr->blk_geom->bheight + bheight_offset_to_8;

                // Align parent block has origin inherited by current block
#if PREDICTIVE_ME
                uint32_t x_to_search = context_ptr->blk_geom->origin_x - (context_ptr->geom_offset_x + ((context_ptr->blk_geom->origin_x & 0x7) ? 4 : 0));
                uint32_t y_to_search = context_ptr->blk_geom->origin_y - (context_ptr->geom_offset_y + ((context_ptr->blk_geom->origin_y & 0x7) ? 4 : 0));
#else
                uint32_t x_to_search = context_ptr->blk_geom->origin_x - (geom_offset_x + ((context_ptr->blk_geom->origin_x & 0x7) ? 4 : 0));
                uint32_t y_to_search = context_ptr->blk_geom->origin_y - (geom_offset_y + ((context_ptr->blk_geom->origin_y & 0x7) ? 4 : 0));
#endif
                // Search the me_info_index of the parent block
                uint32_t me_info_index = 0;

#if PREDICTIVE_ME
                for (uint32_t block_index = 0; block_index < picture_control_set_ptr->parent_pcs_ptr->max_number_of_pus_per_sb; block_index++) {
#else
                for (uint32_t block_index = 0; block_index < max_number_of_pus_per_sb; block_index++) {
#endif

                    if (
                        (bwidth_to_search == partition_width[block_index]) &&
                        (bheight_to_search == partition_height[block_index]) &&
                        (x_to_search == pu_search_index_map[block_index][0]) &&
                        (y_to_search == pu_search_index_map[block_index][1]))
                    {
                        me_info_index = block_index;
                        break;
                    }
                }

                inject_new_candidates(
                    sequence_control_set_ptr,
                    context_ptr,
                    picture_control_set_ptr,
                    isCompoundEnabled,
                    allow_bipred,
                    context_ptr->me_sb_addr,
                    ss_mecontext,
                    use_close_loop_me,
                    close_loop_me_index,
                    me_info_index,
                    &canTotalCnt);
            }
        }
#else
        /**************
            NEWMV L0
        ************* */
        for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; ++me_candidate_index)
        {
            const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
            const uint8_t inter_direction = me_block_results_ptr->direction;
            const uint8_t list0_ref_index = me_block_results_ptr->ref_idx_l0;
            const uint8_t list1_ref_index = me_block_results_ptr->ref_idx_l1;

            if (inter_direction == 0) {
#if MEMORY_FOOTPRINT_OPT_ME_MV
                int16_t to_inject_mv_x = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].x_mv << 1;
                int16_t to_inject_mv_y = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].y_mv << 1;
#else
                int16_t to_inject_mv_x = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_block_results_ptr->x_mv_l0 << 1;
                int16_t to_inject_mv_y = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_block_results_ptr->y_mv_l0 << 1;
#endif
#if MRP_DUPLICATION_FIX
                uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                if (context_ptr->injected_mv_count_l0 == 0 || mrp_is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE) {
#else
                if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
#endif

                    candidateArray[canTotalCnt].type = INTER_MODE;
                    candidateArray[canTotalCnt].distortion_ready = 0;
                    candidateArray[canTotalCnt].use_intrabc = 0;
                    candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                    candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)0;
                    candidateArray[canTotalCnt].inter_mode = NEWMV;
                    candidateArray[canTotalCnt].pred_mode = NEWMV;
                    candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;

                    candidateArray[canTotalCnt].is_compound = 0;
                    candidateArray[canTotalCnt].is_new_mv = 1;
                    candidateArray[canTotalCnt].is_zero_mv = 0;

                    candidateArray[canTotalCnt].drl_index = 0;

                    // Set the MV to ME result
                    candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x;
                    candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y;

                    // will be needed later by the rate estimation
                    candidateArray[canTotalCnt].ref_mv_index = 0;
                    candidateArray[canTotalCnt].pred_mv_weight = 0;
#if MRP_LIST_REF_IDX_TYPE_LT
                    candidateArray[canTotalCnt].ref_frame_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                    candidateArray[canTotalCnt].ref_frame_index_l0 = list0_ref_index;
                    candidateArray[canTotalCnt].ref_frame_index_l1 = -1;
#else
                    candidateArray[canTotalCnt].ref_frame_type = LAST_FRAME;
#endif

#if ATB_TX_TYPE_SUPPORT_PER_TU
                    candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                    candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
                    candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                    candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

                    ChooseBestAv1MvPred(
                        context_ptr,
                        candidateArray[canTotalCnt].md_rate_estimation_ptr,
                        context_ptr->cu_ptr,
                        candidateArray[canTotalCnt].ref_frame_type,
                        candidateArray[canTotalCnt].is_compound,
                        candidateArray[canTotalCnt].pred_mode,
                        candidateArray[canTotalCnt].motion_vector_xl0,
                        candidateArray[canTotalCnt].motion_vector_yl0,
                        0, 0,
                        &candidateArray[canTotalCnt].drl_index,
                        bestPredmv);

                    candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                    candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;

#if CHECK_CAND
                    INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
                    ++canTotalCnt;
#endif

                    context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
                    context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
#if MRP_DUPLICATION_FIX
                    context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] = to_inject_ref_type;
#endif
                    ++context_ptr->injected_mv_count_l0;
                }
            }

            if (isCompoundEnabled) {
                /**************
                   NEWMV L1
               ************* */
                if (inter_direction == 1) {
#if MEMORY_FOOTPRINT_OPT_ME_MV
#if FROM_7_TO_4_MV
                    int16_t to_inject_mv_x = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2) + list1_ref_index].x_mv << 1;
                    int16_t to_inject_mv_y = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2) + list1_ref_index].y_mv << 1;
#else
                    int16_t to_inject_mv_x = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][4 + list1_ref_index].x_mv << 1;
                    int16_t to_inject_mv_y = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][4 + list1_ref_index].y_mv << 1;
#endif
#else
                    int16_t to_inject_mv_x = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_block_results_ptr->x_mv_l1 << 1;
                    int16_t to_inject_mv_y = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_block_results_ptr->y_mv_l1 << 1;
#endif
#if MRP_DUPLICATION_FIX
                    uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
                    if (context_ptr->injected_mv_count_l1 == 0 || mrp_is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE) {
#else
                    if (context_ptr->injected_mv_count_l1 == 0 || is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
#endif

                        candidateArray[canTotalCnt].type = INTER_MODE;
                        candidateArray[canTotalCnt].distortion_ready = 0;
                        candidateArray[canTotalCnt].use_intrabc = 0;
                        candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                        candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)1;

                        candidateArray[canTotalCnt].inter_mode = NEWMV;
                        candidateArray[canTotalCnt].pred_mode = NEWMV;
                        candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;

                        candidateArray[canTotalCnt].is_compound = 0;
                        candidateArray[canTotalCnt].is_new_mv = 1;
                        candidateArray[canTotalCnt].is_zero_mv = 0;

                        candidateArray[canTotalCnt].drl_index = 0;

                        // Set the MV to ME result
                        candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x;
                        candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y;

                        // will be needed later by the rate estimation
                        candidateArray[canTotalCnt].ref_mv_index = 0;
                        candidateArray[canTotalCnt].pred_mv_weight = 0;
#if MRP_LIST_REF_IDX_TYPE_LT
                        candidateArray[canTotalCnt].ref_frame_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
                        candidateArray[canTotalCnt].ref_frame_index_l0 = -1;
                        candidateArray[canTotalCnt].ref_frame_index_l1 = list1_ref_index;
#else
                        candidateArray[canTotalCnt].ref_frame_type = BWDREF_FRAME;
#endif

#if ATB_TX_TYPE_SUPPORT_PER_TU
                        candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                        candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
                        candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                        candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif
                        ChooseBestAv1MvPred(
                            context_ptr,
                            candidateArray[canTotalCnt].md_rate_estimation_ptr,
                            context_ptr->cu_ptr,
                            candidateArray[canTotalCnt].ref_frame_type,
                            candidateArray[canTotalCnt].is_compound,
                            candidateArray[canTotalCnt].pred_mode,
                            candidateArray[canTotalCnt].motion_vector_xl1,
                            candidateArray[canTotalCnt].motion_vector_yl1,
                            0, 0,
                            &candidateArray[canTotalCnt].drl_index,
                            bestPredmv);

                        candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[0].as_mv.col;
                        candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[0].as_mv.row;
#if CHECK_CAND
                        INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
                        ++canTotalCnt;
#endif

                        context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_x;
                        context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_y;
#if MRP_DUPLICATION_FIX
                        context_ptr->injected_ref_type_l1_array[context_ptr->injected_mv_count_l1] = to_inject_ref_type;
#endif
                        ++context_ptr->injected_mv_count_l1;
                    }
                }
                /**************
                   NEW_NEWMV
                ************* */
                if (allow_bipred) {
                    if (inter_direction == 2) {
#if MEMORY_FOOTPRINT_OPT_ME_MV
                        int16_t to_inject_mv_x_l0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].x_mv << 1;
                        int16_t to_inject_mv_y_l0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][list0_ref_index].y_mv << 1;
#if FROM_7_TO_4_MV
                        int16_t to_inject_mv_x_l1 = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].x_mv << 1;
                        int16_t to_inject_mv_y_l1 = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results_ptr->ref1_list << 2) : (me_block_results_ptr->ref1_list << 1)) + list1_ref_index].y_mv << 1;
#else
                        int16_t to_inject_mv_x_l1 = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][(me_block_results_ptr->ref1_list << 2) + list1_ref_index].x_mv << 1;
                        int16_t to_inject_mv_y_l1 = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_results->me_mv_array[context_ptr->me_block_offset][(me_block_results_ptr->ref1_list << 2) + list1_ref_index].y_mv << 1;
#endif
#else
                        int16_t to_inject_mv_x_l0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : me_block_results_ptr->x_mv_l0 << 1;
                        int16_t to_inject_mv_y_l0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : me_block_results_ptr->y_mv_l0 << 1;
                        int16_t to_inject_mv_x_l1 = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : me_block_results_ptr->x_mv_l1 << 1;
                        int16_t to_inject_mv_y_l1 = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : me_block_results_ptr->y_mv_l1 << 1;
#endif
#if MRP_DUPLICATION_FIX
                        MvReferenceFrame rf[2];
#if MRP_MD_UNI_DIR_BIPRED
                        rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
                        rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
#else
                        rf[0] = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                        rf[1] = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#endif
                        uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
                        if (context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, to_inject_ref_type) == EB_FALSE) {
#else
                        if (context_ptr->injected_mv_count_bipred == 0 || is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1) == EB_FALSE) {
#endif

                            candidateArray[canTotalCnt].type = INTER_MODE;

                            candidateArray[canTotalCnt].distortion_ready = 0;
                            candidateArray[canTotalCnt].use_intrabc = 0;

                            candidateArray[canTotalCnt].merge_flag = EB_FALSE;

                            candidateArray[canTotalCnt].is_new_mv = 1;
                            candidateArray[canTotalCnt].is_zero_mv = 0;

                            candidateArray[canTotalCnt].drl_index = 0;

                            // Set the MV to ME result

                            candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x_l0;
                            candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y_l0;
                            candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x_l1;
                            candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y_l1;

                            // will be needed later by the rate estimation
                            candidateArray[canTotalCnt].ref_mv_index = 0;
                            candidateArray[canTotalCnt].pred_mv_weight = 0;

                            candidateArray[canTotalCnt].inter_mode = NEW_NEWMV;
                            candidateArray[canTotalCnt].pred_mode = NEW_NEWMV;
                            candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                            candidateArray[canTotalCnt].is_compound = 1;
                            candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)2;
#if MRP_LIST_REF_IDX_TYPE_LT
                            MvReferenceFrame rf[2];
#if MRP_MD_UNI_DIR_BIPRED
                            rf[0] = svt_get_ref_frame_type(me_block_results_ptr->ref0_list, list0_ref_index);
                            rf[1] = svt_get_ref_frame_type(me_block_results_ptr->ref1_list, list1_ref_index);
#else
                            rf[0] = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                            rf[1] = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
#endif
                            candidateArray[canTotalCnt].ref_frame_type = av1_ref_frame_type(rf);

                            candidateArray[canTotalCnt].ref_frame_index_l0 = list0_ref_index;
                            candidateArray[canTotalCnt].ref_frame_index_l1 = list1_ref_index;
#else
                            candidateArray[canTotalCnt].ref_frame_type = LAST_BWD_FRAME;
#endif

#if ATB_TX_TYPE_SUPPORT_PER_TU
                            candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                            candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
                            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif

                            ChooseBestAv1MvPred(
                                context_ptr,
                                candidateArray[canTotalCnt].md_rate_estimation_ptr,
                                context_ptr->cu_ptr,
                                candidateArray[canTotalCnt].ref_frame_type,
                                candidateArray[canTotalCnt].is_compound,
                                candidateArray[canTotalCnt].pred_mode,
                                candidateArray[canTotalCnt].motion_vector_xl0,
                                candidateArray[canTotalCnt].motion_vector_yl0,
                                candidateArray[canTotalCnt].motion_vector_xl1,
                                candidateArray[canTotalCnt].motion_vector_yl1,
                                &candidateArray[canTotalCnt].drl_index,
                                bestPredmv);

                            candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                            candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
                            candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[1].as_mv.col;
                            candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[1].as_mv.row;
#if CHECK_CAND
                            INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
                            ++canTotalCnt;
#endif

                            context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                            context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                            context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                            context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
#if MRP_DUPLICATION_FIX
                            context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
#endif
                            ++context_ptr->injected_mv_count_bipred;
                        }
                    }
                }
            }
        }
#endif
    }

    if (context_ptr->global_mv_injection) {
        /**************
         GLOBALMV L0
        ************* */
        {
            int16_t to_inject_mv_x = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[1] >> GM_TRANS_ONLY_PREC_DIFF);
            int16_t to_inject_mv_y = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[0] >> GM_TRANS_ONLY_PREC_DIFF);
#if MRP_DUPLICATION_FIX
            uint8_t to_inject_ref_type = svt_get_ref_frame_type(REF_LIST_0, 0/*list0_ref_index*/);
            if (context_ptr->injected_mv_count_l0 == 0 || mrp_is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y, to_inject_ref_type) == EB_FALSE) {
#else
            if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
#endif
#if II_SEARCH      // GLOBALMV L0
             MvReferenceFrame rf[2];
             rf[0] = to_inject_ref_type;
             rf[1] = -1;
            uint8_t tot_ii_types = svt_is_interintra_allowed(picture_control_set_ptr->parent_pcs_ptr->enable_inter_intra,bsize, GLOBALMV, rf) ? II_COUNT : 1;
            uint8_t ii_type;
            for (ii_type = 0; ii_type < tot_ii_types; ii_type++)
            {
#endif
                candidateArray[canTotalCnt].type = INTER_MODE;

                candidateArray[canTotalCnt].distortion_ready = 0;
                candidateArray[canTotalCnt].use_intrabc = 0;

                candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)0;

                candidateArray[canTotalCnt].inter_mode = GLOBALMV;
                candidateArray[canTotalCnt].pred_mode = GLOBALMV;
                candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                candidateArray[canTotalCnt].is_compound = 0;
                candidateArray[canTotalCnt].is_new_mv = 0;
                candidateArray[canTotalCnt].is_zero_mv = 0;
                candidateArray[canTotalCnt].drl_index = 0;

                // will be needed later by the rate estimation
                candidateArray[canTotalCnt].ref_mv_index = 0;
                candidateArray[canTotalCnt].pred_mv_weight = 0;
                candidateArray[canTotalCnt].ref_frame_type = LAST_FRAME;
#if MRP_LIST_REF_IDX_TYPE_LT
                candidateArray[canTotalCnt].ref_frame_index_l0 = 0;
                candidateArray[canTotalCnt].ref_frame_index_l1 = -1;
#endif

#if ATB_TX_TYPE_SUPPORT_PER_TU
                candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif
                // Set the MV to frame MV
                candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x;
                candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y;
#if II_SEARCH
                candidateArray[canTotalCnt].is_interintra_used = ii_type == 0 ? 0 : 1;
                if (ii_type == 1) {
                    inter_intra_search(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canTotalCnt]);

                    candidateArray[canTotalCnt].use_wedge_interintra = 1;
                    candidateArray[canTotalCnt].ii_wedge_sign = 0;

                }
                if (ii_type == 2)// smooth
                {
                    candidateArray[canTotalCnt].interintra_mode = candidateArray[canTotalCnt-1].interintra_mode;
                    candidateArray[canTotalCnt].use_wedge_interintra = 0;
                }
#endif

#if CHECK_CAND
                INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
                ++canTotalCnt;
#endif
#if II_SEARCH
            }
#endif

                context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
                context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
#if MRP_DUPLICATION_FIX
                context_ptr->injected_ref_type_l0_array[context_ptr->injected_mv_count_l0] = to_inject_ref_type;
#endif
                ++context_ptr->injected_mv_count_l0;
            }
            }

        if (isCompoundEnabled && allow_bipred) {
            /**************
            GLOBAL_GLOBALMV
            ************* */

            int16_t to_inject_mv_x_l0 = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[1] >> GM_TRANS_ONLY_PREC_DIFF);
            int16_t to_inject_mv_y_l0 = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[0] >> GM_TRANS_ONLY_PREC_DIFF);
            int16_t to_inject_mv_x_l1 = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[BWDREF_FRAME].wmmat[1] >> GM_TRANS_ONLY_PREC_DIFF);
            int16_t to_inject_mv_y_l1 = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[BWDREF_FRAME].wmmat[0] >> GM_TRANS_ONLY_PREC_DIFF);
#if MRP_DUPLICATION_FIX
            MvReferenceFrame rf[2];
            rf[0] = svt_get_ref_frame_type(REF_LIST_0, 0/*list0_ref_index*/);
            rf[1] = svt_get_ref_frame_type(REF_LIST_1, 0/*list1_ref_index*/);
            uint8_t to_inject_ref_type = av1_ref_frame_type(rf);
            if (context_ptr->injected_mv_count_bipred == 0 || mrp_is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1, to_inject_ref_type) == EB_FALSE) {
#else
            if (context_ptr->injected_mv_count_bipred == 0 || is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1) == EB_FALSE) {
#endif

#if COMP_MODE
                context_ptr->variance_ready = 0 ;
                for (cur_type = MD_COMP_AVG; cur_type <= tot_comp_types ; cur_type++)
                {
                // If two predictors are very similar, skip wedge compound mode search
                if (context_ptr->variance_ready)
                    if (context_ptr->prediction_mse < 8 || (!have_newmv_in_inter_mode(GLOBAL_GLOBALMV) && context_ptr->prediction_mse  < 64))
                        continue;
#endif
                candidateArray[canTotalCnt].type = INTER_MODE;
                candidateArray[canTotalCnt].distortion_ready = 0;
                candidateArray[canTotalCnt].use_intrabc = 0;

                candidateArray[canTotalCnt].merge_flag = EB_FALSE;

                candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)2;

                candidateArray[canTotalCnt].inter_mode = GLOBAL_GLOBALMV;
                candidateArray[canTotalCnt].pred_mode = GLOBAL_GLOBALMV;
                candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                candidateArray[canTotalCnt].is_compound = 1;
#if II_COMP
                candidateArray[canTotalCnt].is_interintra_used = 0;
#endif
                candidateArray[canTotalCnt].is_new_mv = 0;
                candidateArray[canTotalCnt].is_zero_mv = 0;
                candidateArray[canTotalCnt].drl_index = 0;

                // will be needed later by the rate estimation
                candidateArray[canTotalCnt].ref_mv_index = 0;
                candidateArray[canTotalCnt].pred_mv_weight = 0;
                candidateArray[canTotalCnt].ref_frame_type = LAST_BWD_FRAME;
#if MRP_LIST_REF_IDX_TYPE_LT
                candidateArray[canTotalCnt].ref_frame_index_l0 = 0;
                candidateArray[canTotalCnt].ref_frame_index_l1 = 0;
#endif
#if ATB_TX_TYPE_SUPPORT_PER_TU
                candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
#else
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif
                // Set the MV to frame MV

                candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x_l0;
                candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y_l0;
                candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x_l1;
                candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y_l1;
#if COMP_MODE
                    //GLOB-GLOB
                    determine_compound_mode(
                        picture_control_set_ptr,
                        context_ptr,
                        &candidateArray[canTotalCnt],
                        cur_type);
#endif

#if CHECK_CAND
                INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
                ++canTotalCnt;
#endif

                context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
#if MRP_DUPLICATION_FIX
                context_ptr->injected_ref_type_bipred_array[context_ptr->injected_mv_count_bipred] = to_inject_ref_type;
#endif
                ++context_ptr->injected_mv_count_bipred;
#if COMP_MODE
                }
#endif
            }
            }
        }

    // Warped Motion
    if (picture_control_set_ptr->parent_pcs_ptr->allow_warped_motion &&
        has_overlappable_candidates(context_ptr->cu_ptr) &&
        context_ptr->blk_geom->bwidth >= 8 &&
        context_ptr->blk_geom->bheight >= 8 &&
        context_ptr->warped_motion_injection) {
        inject_warped_motion_candidates(
            picture_control_set_ptr,
            context_ptr,
            context_ptr->cu_ptr,
            &canTotalCnt,
            ss_mecontext,
            me_results,
#if !MEMORY_FOOTPRINT_OPT_ME_MV
            me2Nx2NTableOffset,
#endif
            use_close_loop_me,
            close_loop_me_index);
    }

    if (inject_newmv_candidate) {
#if BASE_LAYER_REF || MRP_REF_MODE
        if (isCompoundEnabled) {
            if (allow_bipred) {
#else
        if (allow_bipred) {
#endif

            //----------------------
            // Bipred2Nx2N
            //----------------------
            if (context_ptr->bipred3x3_injection > 0)
                if (picture_control_set_ptr->slice_type == B_SLICE)
                    Bipred3x3CandidatesInjection(
#if MEMORY_FOOTPRINT_OPT_ME_MV
                        sequence_control_set_ptr,
#endif
                        picture_control_set_ptr,
                        context_ptr,
                        sb_ptr,
#if NEW_NEAREST_NEW_INJECTION
                        context_ptr->me_sb_addr,
#else
                        me_sb_addr,
#endif
                        ss_mecontext,
                        use_close_loop_me,
                        close_loop_me_index,
#if !MEMORY_FOOTPRINT_OPT_ME_MV
                        me2Nx2NTableOffset,
#endif
                        &canTotalCnt);

#if BASE_LAYER_REF || MRP_REF_MODE
        }
#endif

        //----------------------
        // Unipred2Nx2N
        //----------------------
        if (context_ptr->unipred3x3_injection > 0)
            if (picture_control_set_ptr->slice_type != I_SLICE)
                Unipred3x3CandidatesInjection(
#if MEMORY_FOOTPRINT_OPT_ME_MV
                    sequence_control_set_ptr,
#endif
                    picture_control_set_ptr,
                    context_ptr,
                    sb_ptr,
#if NEW_NEAREST_NEW_INJECTION
                    context_ptr->me_sb_addr,
#else
                    me_sb_addr,
#endif
                    ss_mecontext,
                    use_close_loop_me,
                    close_loop_me_index,
#if !MEMORY_FOOTPRINT_OPT_ME_MV
                    me2Nx2NTableOffset,
#endif
                    &canTotalCnt);
            }
        }

#if 0//EIGTH_PEL_MV
        //----------------------
        // Eighth-pel refinement
        //----------------------
        if (inject_newmv_candidate && picture_control_set_ptr->parent_pcs_ptr->allow_high_precision_mv) {
#if BASE_LAYER_REF
            if (isCompoundEnabled) {
                if (allow_bipred) {
#else
            if (allow_bipred) {
#endif
#if IMPROVED_BIPRED_INJECTION
                //----------------------
                // Inject eight-pel bi-pred
                //----------------------
                if (context_ptr->bipred3x3_injection > 0)
                    if (picture_control_set_ptr->slice_type == B_SLICE)
                        eighth_pel_bipred_refinement(
                            sequence_control_set_ptr,
                            picture_control_set_ptr,
                            context_ptr,
                            context_ptr->me_sb_addr,
                            ss_mecontext,
                            use_close_loop_me,
                            close_loop_me_index,
                            &canTotalCnt);

#endif
#if BASE_LAYER_REF
            }
#endif
#if IMPROVED_UNIPRED_INJECTION
            //----------------------
            // Inject eight-pel uni-pred
            //----------------------
            if (context_ptr->unipred3x3_injection > 0)
                if (picture_control_set_ptr->slice_type != I_SLICE)
                    eighth_pel_unipred_refinement(
                        sequence_control_set_ptr,
                        picture_control_set_ptr,
                        context_ptr,
                        context_ptr->me_sb_addr,
                        ss_mecontext,
                        use_close_loop_me,
                        close_loop_me_index,
                        &canTotalCnt);
#endif
                }
            }
#endif
#if PREDICTIVE_ME // inject them
            if (context_ptr->predictive_me_level)
                inject_predictive_me_candidates(
                    //sequence_control_set_ptr,
                    context_ptr,
                    picture_control_set_ptr,
                    isCompoundEnabled,
                    allow_bipred,
                    &canTotalCnt);
#endif
// update the total number of candidates injected
(*candidateTotalCnt) = canTotalCnt;

return;
    }

#else
// CHECK_CAND (x6)
// END of Function Declarations
void  inject_inter_candidates(
    PictureControlSet            *picture_control_set_ptr,
    ModeDecisionContext          *context_ptr,
    SsMeContext                  *ss_mecontext,
    const SequenceControlSet     *sequence_control_set_ptr,
    LargestCodingUnit            *sb_ptr,
#if M8_SKIP_BLK
    uint32_t                       *candidateTotalCnt){
#else
    uint32_t                       *candidateTotalCnt,
    const uint32_t                  leaf_index){
#endif

    (void)sequence_control_set_ptr;
    uint32_t                   canTotalCnt = *candidateTotalCnt;
    const uint32_t             lcuAddr = sb_ptr->index;
    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
    static MvReferenceFrame ref_frames[] = { LAST_FRAME, BWDREF_FRAME, LAST_BWD_FRAME };
    EbBool isCompoundEnabled = (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
    uint32_t me_sb_addr;
    uint32_t geom_offset_x = 0;
    uint32_t geom_offset_y = 0;

    if (sequence_control_set_ptr->sb_size == BLOCK_128X128) {
        uint32_t me_sb_size = sequence_control_set_ptr->sb_sz;
        uint32_t me_pic_width_in_sb = (sequence_control_set_ptr->seq_header.frame_width_bits + sequence_control_set_ptr->sb_sz - 1) / me_sb_size;
        uint32_t me_sb_x = (context_ptr->cu_origin_x / me_sb_size);
        uint32_t me_sb_y = (context_ptr->cu_origin_y / me_sb_size);

        me_sb_addr = me_sb_x + me_sb_y * me_pic_width_in_sb;

        geom_offset_x = (me_sb_x & 0x1) * me_sb_size;
        geom_offset_y = (me_sb_y & 0x1) * me_sb_size;
    }
    else
        me_sb_addr = lcuAddr;
    uint32_t max_number_of_pus_per_sb;

    max_number_of_pus_per_sb = picture_control_set_ptr->parent_pcs_ptr->max_number_of_pus_per_sb;

    uint32_t me2Nx2NTableOffset;

        me2Nx2NTableOffset = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4 || context_ptr->blk_geom->bwidth == 128 || context_ptr->blk_geom->bheight == 128) ? 0 :
            get_me_info_index(max_number_of_pus_per_sb, context_ptr->blk_geom, geom_offset_x, geom_offset_y);

    MeCuResults * mePuResult = &picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr][me2Nx2NTableOffset];
    EbBool use_close_loop_me = picture_control_set_ptr->parent_pcs_ptr->enable_in_loop_motion_estimation_flag &&
        ((context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) || (context_ptr->blk_geom->bwidth > 64 || context_ptr->blk_geom->bheight > 64)) ? EB_TRUE : EB_FALSE;

    uint32_t close_loop_me_index = use_close_loop_me ? get_in_loop_me_info_index(MAX_SS_ME_PU_COUNT, sequence_control_set_ptr->sb_size == BLOCK_128X128 ? 1 : 0, context_ptr->blk_geom) : 0;
#if BASE_LAYER_REF || MRP_REF_MODE
    EbBool allow_bipred = (picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0 || context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) ? EB_FALSE : EB_TRUE;
#else
    EbBool allow_bipred = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) ? EB_FALSE : EB_TRUE;
#endif
    IntMv  bestPredmv[2] = { {0}, {0} };
    uint8_t sq_index = LOG2F(context_ptr->blk_geom->sq_size) - 2;
    uint8_t inject_newmv_candidate = 1;
    if (picture_control_set_ptr->parent_pcs_ptr->nsq_search_level >= NSQ_SEARCH_LEVEL1 &&
        picture_control_set_ptr->parent_pcs_ptr->nsq_search_level < NSQ_SEARCH_FULL) {
        inject_newmv_candidate = context_ptr->blk_geom->shape == PART_N ? 1 :
            context_ptr->parent_sq_has_coeff[sq_index] != 0 ? inject_newmv_candidate : 0;
    }

    generate_av1_mvp_table(
        &sb_ptr->tile_info,
        context_ptr,
        context_ptr->cu_ptr,
        context_ptr->blk_geom,
        context_ptr->cu_origin_x,
        context_ptr->cu_origin_y,
        ref_frames,
        (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) ? 1 : 3,
        picture_control_set_ptr);

    uint32_t mi_row = context_ptr->cu_origin_y >> MI_SIZE_LOG2;
    uint32_t mi_col = context_ptr->cu_origin_x >> MI_SIZE_LOG2;
    av1_count_overlappable_neighbors(
        picture_control_set_ptr,
        context_ptr->cu_ptr,
        context_ptr->blk_geom->bsize,
        mi_row,
        mi_col);

    /**************
         MVP
    ************* */
    InjectAv1MvpCandidates(
        context_ptr,
        context_ptr->cu_ptr,
        ref_frames,
        picture_control_set_ptr,
        lcuAddr,
#if !M8_SKIP_BLK
        leaf_index,
#endif
        allow_bipred,
        &canTotalCnt);

    if (inject_newmv_candidate) {
#if M9_INTER_SRC_SRC_FAST_LOOP
        // Derive PA distortion(s) per direction
        uint32_t distortion_l0 = ~0;
        uint32_t distortion_l1 = ~0;
        uint32_t distortion_bipred = ~0;
        for (uint8_t total_me_candidate_index = 0; total_me_candidate_index < mePuResult->total_me_candidate_index; total_me_candidate_index++) {
            switch (mePuResult->distortion_direction[total_me_candidate_index].direction) {
            case UNI_PRED_LIST_0:
                distortion_l0 = mePuResult->distortion_direction[total_me_candidate_index].distortion;
                break;
            case UNI_PRED_LIST_1:
                distortion_l1 = mePuResult->distortion_direction[total_me_candidate_index].distortion;
                break;
            case BI_PRED:
                distortion_bipred = mePuResult->distortion_direction[total_me_candidate_index].distortion;
                break;
            }
        }

         /**************
            NEWMV L0
        ************* */
        int16_t to_inject_mv_x = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : mePuResult->x_mv_l0 << 1;
        int16_t to_inject_mv_y = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : mePuResult->y_mv_l0 << 1;
        if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
            candidateArray[canTotalCnt].type = INTER_MODE;
            candidateArray[canTotalCnt].distortion_ready = context_ptr->inter_fast_loop_src_src ? 1 : 0;
            candidateArray[canTotalCnt].me_distortion = distortion_l0;
            candidateArray[canTotalCnt].use_intrabc = 0;
            candidateArray[canTotalCnt].merge_flag = EB_FALSE;
            candidateArray[canTotalCnt].prediction_direction[0] = UNI_PRED_LIST_0;
            candidateArray[canTotalCnt].inter_mode = NEWMV;
            candidateArray[canTotalCnt].pred_mode = NEWMV;
            candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;

            candidateArray[canTotalCnt].is_compound = 0;
            candidateArray[canTotalCnt].is_new_mv = 1;
            candidateArray[canTotalCnt].is_zero_mv = 0;

            candidateArray[canTotalCnt].drl_index = 0;

            // Set the MV to ME result
            candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x;
            candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y;
            // will be needed later by the rate estimation
            candidateArray[canTotalCnt].ref_mv_index = 0;
            candidateArray[canTotalCnt].pred_mv_weight = 0;
            candidateArray[canTotalCnt].ref_frame_type = LAST_FRAME;

            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

            ChooseBestAv1MvPred(
                context_ptr,
                candidateArray[canTotalCnt].md_rate_estimation_ptr,
                context_ptr->cu_ptr,
                candidateArray[canTotalCnt].ref_frame_type,
                candidateArray[canTotalCnt].is_compound,
                candidateArray[canTotalCnt].pred_mode,
                candidateArray[canTotalCnt].motion_vector_xl0,
                candidateArray[canTotalCnt].motion_vector_yl0,
                0, 0,
                &candidateArray[canTotalCnt].drl_index,
                bestPredmv);

            candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
            candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;

            ++canTotalCnt;
            context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
            context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
            ++context_ptr->injected_mv_count_l0;
        }
        if (isCompoundEnabled) {
            /**************
               NEWMV L1
           ************* */
            int16_t to_inject_mv_x = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : mePuResult->x_mv_l1 << 1;
            int16_t to_inject_mv_y = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : mePuResult->y_mv_l1 << 1;
            if (context_ptr->injected_mv_count_l1 == 0 || is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
                candidateArray[canTotalCnt].type = INTER_MODE;
                candidateArray[canTotalCnt].distortion_ready = context_ptr->inter_fast_loop_src_src ? 1 : 0;
                candidateArray[canTotalCnt].me_distortion = distortion_l1;
                candidateArray[canTotalCnt].use_intrabc = 0;
                candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                candidateArray[canTotalCnt].prediction_direction[0] = UNI_PRED_LIST_1;
                candidateArray[canTotalCnt].inter_mode = NEWMV;
                candidateArray[canTotalCnt].pred_mode = NEWMV;
                candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;

                candidateArray[canTotalCnt].is_compound = 0;
                candidateArray[canTotalCnt].is_new_mv = 1;
                candidateArray[canTotalCnt].is_zero_mv = 0;

                candidateArray[canTotalCnt].drl_index = 0;

                // Set the MV to ME result
                candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x;
                candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y;
                // will be needed later by the rate estimation
                candidateArray[canTotalCnt].ref_mv_index = 0;
                candidateArray[canTotalCnt].pred_mv_weight = 0;
                candidateArray[canTotalCnt].ref_frame_type = BWDREF_FRAME;

                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

                ChooseBestAv1MvPred(
                    context_ptr,
                    candidateArray[canTotalCnt].md_rate_estimation_ptr,
                    context_ptr->cu_ptr,
                    candidateArray[canTotalCnt].ref_frame_type,
                    candidateArray[canTotalCnt].is_compound,
                    candidateArray[canTotalCnt].pred_mode,
                    candidateArray[canTotalCnt].motion_vector_xl1,
                    candidateArray[canTotalCnt].motion_vector_yl1,
                    0, 0,
                    &candidateArray[canTotalCnt].drl_index,
                    bestPredmv);

                candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[0].as_mv.col;
                candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[0].as_mv.row;

                ++canTotalCnt;
                context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_x;
                context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_y;
                ++context_ptr->injected_mv_count_l1;
            }
            /**************
               NEW_NEWMV
            ************* */
            if (allow_bipred) {
                int16_t to_inject_mv_x_l0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : mePuResult->x_mv_l0 << 1;
                int16_t to_inject_mv_y_l0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : mePuResult->y_mv_l0 << 1;
                int16_t to_inject_mv_x_l1 = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : mePuResult->x_mv_l1 << 1;
                int16_t to_inject_mv_y_l1 = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : mePuResult->y_mv_l1 << 1;
                if (context_ptr->injected_mv_count_bipred == 0 || is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1) == EB_FALSE) {
                    candidateArray[canTotalCnt].type = INTER_MODE;
                    candidateArray[canTotalCnt].distortion_ready = context_ptr->inter_fast_loop_src_src ? 1 : 0;
                    candidateArray[canTotalCnt].me_distortion = distortion_bipred;
                    candidateArray[canTotalCnt].use_intrabc = 0;
                    candidateArray[canTotalCnt].merge_flag = EB_FALSE;

                    candidateArray[canTotalCnt].is_new_mv = 1;
                    candidateArray[canTotalCnt].is_zero_mv = 0;

                    candidateArray[canTotalCnt].drl_index = 0;

                    // Set the MV to ME result
                    candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x_l0;
                    candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y_l0;
                    candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x_l1;
                    candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y_l1;
                    // will be needed later by the rate estimation
                    candidateArray[canTotalCnt].ref_mv_index = 0;
                    candidateArray[canTotalCnt].pred_mv_weight = 0;

                    candidateArray[canTotalCnt].inter_mode = NEW_NEWMV;
                    candidateArray[canTotalCnt].pred_mode = NEW_NEWMV;
                    candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                    candidateArray[canTotalCnt].is_compound = 1;
                    candidateArray[canTotalCnt].prediction_direction[0] = BI_PRED;
                    candidateArray[canTotalCnt].ref_frame_type = LAST_BWD_FRAME;

                    candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                    candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

                    ChooseBestAv1MvPred(
                        context_ptr,
                        candidateArray[canTotalCnt].md_rate_estimation_ptr,
                        context_ptr->cu_ptr,
                        candidateArray[canTotalCnt].ref_frame_type,
                        candidateArray[canTotalCnt].is_compound,
                        candidateArray[canTotalCnt].pred_mode,
                        candidateArray[canTotalCnt].motion_vector_xl0,
                        candidateArray[canTotalCnt].motion_vector_yl0,
                        candidateArray[canTotalCnt].motion_vector_xl1,
                        candidateArray[canTotalCnt].motion_vector_yl1,
                        &candidateArray[canTotalCnt].drl_index,
                        bestPredmv);

                    candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                    candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
                    candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[1].as_mv.col;
                    candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[1].as_mv.row;

                    ++canTotalCnt;
                    context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                    context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                    context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                    context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                    ++context_ptr->injected_mv_count_bipred;
                }
            }
        }
#else
        /**************
            NEWMV L0
        ************* */
        int16_t to_inject_mv_x = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : mePuResult->x_mv_l0 << 1;
        int16_t to_inject_mv_y = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : mePuResult->y_mv_l0 << 1;
        if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
        candidateArray[canTotalCnt].type = INTER_MODE;
        candidateArray[canTotalCnt].distortion_ready = 0;
        candidateArray[canTotalCnt].use_intrabc = 0;
        candidateArray[canTotalCnt].merge_flag = EB_FALSE;
        candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)0;
        candidateArray[canTotalCnt].inter_mode = NEWMV;
        candidateArray[canTotalCnt].pred_mode = NEWMV;
        candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;

        candidateArray[canTotalCnt].is_compound = 0;
        candidateArray[canTotalCnt].is_new_mv = 1;
        candidateArray[canTotalCnt].is_zero_mv = 0;

        candidateArray[canTotalCnt].drl_index = 0;

        // Set the MV to ME result
        candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x;
        candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y;
        // will be needed later by the rate estimation
        candidateArray[canTotalCnt].ref_mv_index = 0;
        candidateArray[canTotalCnt].pred_mv_weight = 0;
        candidateArray[canTotalCnt].ref_frame_type = LAST_FRAME;

        candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
        candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

        ChooseBestAv1MvPred(
            context_ptr,
            candidateArray[canTotalCnt].md_rate_estimation_ptr,
            context_ptr->cu_ptr,
            candidateArray[canTotalCnt].ref_frame_type,
            candidateArray[canTotalCnt].is_compound,
            candidateArray[canTotalCnt].pred_mode,
            candidateArray[canTotalCnt].motion_vector_xl0,
            candidateArray[canTotalCnt].motion_vector_yl0,
            0, 0,
            &candidateArray[canTotalCnt].drl_index,
            bestPredmv);

        candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
        candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;

        ++canTotalCnt;
        context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
        context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
        ++context_ptr->injected_mv_count_l0;
        }
        if (isCompoundEnabled) {
            /**************
               NEWMV L1
           ************* */
            int16_t to_inject_mv_x = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : mePuResult->x_mv_l1 << 1;
            int16_t to_inject_mv_y = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : mePuResult->y_mv_l1 << 1;
            if (context_ptr->injected_mv_count_l1 == 0 || is_already_injected_mv_l1(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
            candidateArray[canTotalCnt].type = INTER_MODE;
            candidateArray[canTotalCnt].distortion_ready = 0;
            candidateArray[canTotalCnt].use_intrabc = 0;
            candidateArray[canTotalCnt].merge_flag = EB_FALSE;
            candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)1;
            candidateArray[canTotalCnt].inter_mode = NEWMV;
            candidateArray[canTotalCnt].pred_mode = NEWMV;
            candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;

            candidateArray[canTotalCnt].is_compound = 0;
            candidateArray[canTotalCnt].is_new_mv = 1;
            candidateArray[canTotalCnt].is_zero_mv = 0;

            candidateArray[canTotalCnt].drl_index = 0;

            // Set the MV to ME result
            candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x;
            candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y;
            // will be needed later by the rate estimation
            candidateArray[canTotalCnt].ref_mv_index = 0;
            candidateArray[canTotalCnt].pred_mv_weight = 0;
            candidateArray[canTotalCnt].ref_frame_type = BWDREF_FRAME;

            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

            ChooseBestAv1MvPred(
                context_ptr,
                candidateArray[canTotalCnt].md_rate_estimation_ptr,
                context_ptr->cu_ptr,
                candidateArray[canTotalCnt].ref_frame_type,
                candidateArray[canTotalCnt].is_compound,
                candidateArray[canTotalCnt].pred_mode,
                candidateArray[canTotalCnt].motion_vector_xl1,
                candidateArray[canTotalCnt].motion_vector_yl1,
                0, 0,
                &candidateArray[canTotalCnt].drl_index,
                bestPredmv);

            candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[0].as_mv.col;
            candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[0].as_mv.row;

            ++canTotalCnt;
            context_ptr->injected_mv_x_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_x;
            context_ptr->injected_mv_y_l1_array[context_ptr->injected_mv_count_l1] = to_inject_mv_y;
            ++context_ptr->injected_mv_count_l1;
            }
            /**************
               NEW_NEWMV
            ************* */
            if (allow_bipred) {
                int16_t to_inject_mv_x_l0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][0] << 1 : mePuResult->x_mv_l0 << 1;
                int16_t to_inject_mv_y_l0 = use_close_loop_me ? ss_mecontext->inloop_me_mv[0][0][close_loop_me_index][1] << 1 : mePuResult->y_mv_l0 << 1;
                int16_t to_inject_mv_x_l1 = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][0] << 1 : mePuResult->x_mv_l1 << 1;
                int16_t to_inject_mv_y_l1 = use_close_loop_me ? ss_mecontext->inloop_me_mv[1][0][close_loop_me_index][1] << 1 : mePuResult->y_mv_l1 << 1;
                if (context_ptr->injected_mv_count_bipred == 0 || is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1) == EB_FALSE) {
                candidateArray[canTotalCnt].type = INTER_MODE;
                candidateArray[canTotalCnt].distortion_ready = 0;
                candidateArray[canTotalCnt].use_intrabc = 0;
                candidateArray[canTotalCnt].merge_flag = EB_FALSE;

                candidateArray[canTotalCnt].is_new_mv = 1;
                candidateArray[canTotalCnt].is_zero_mv = 0;

                candidateArray[canTotalCnt].drl_index = 0;

                // Set the MV to ME result
                candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x_l0;
                candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y_l0;
                candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x_l1;
                candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y_l1;
                // will be needed later by the rate estimation
                candidateArray[canTotalCnt].ref_mv_index = 0;
                candidateArray[canTotalCnt].pred_mv_weight = 0;

                candidateArray[canTotalCnt].inter_mode = NEW_NEWMV;
                candidateArray[canTotalCnt].pred_mode = NEW_NEWMV;
                candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                candidateArray[canTotalCnt].is_compound = 1;
                candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)2;
                candidateArray[canTotalCnt].ref_frame_type = LAST_BWD_FRAME;

                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

                ChooseBestAv1MvPred(
                    context_ptr,
                    candidateArray[canTotalCnt].md_rate_estimation_ptr,
                    context_ptr->cu_ptr,
                    candidateArray[canTotalCnt].ref_frame_type,
                    candidateArray[canTotalCnt].is_compound,
                    candidateArray[canTotalCnt].pred_mode,
                    candidateArray[canTotalCnt].motion_vector_xl0,
                    candidateArray[canTotalCnt].motion_vector_yl0,
                    candidateArray[canTotalCnt].motion_vector_xl1,
                    candidateArray[canTotalCnt].motion_vector_yl1,
                    &candidateArray[canTotalCnt].drl_index,
                    bestPredmv);

                candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_0] = bestPredmv[0].as_mv.col;
                candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_0] = bestPredmv[0].as_mv.row;
                candidateArray[canTotalCnt].motion_vector_pred_x[REF_LIST_1] = bestPredmv[1].as_mv.col;
                candidateArray[canTotalCnt].motion_vector_pred_y[REF_LIST_1] = bestPredmv[1].as_mv.row;

                ++canTotalCnt;
                context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                ++context_ptr->injected_mv_count_bipred;
                }
            }
        }
#endif
    }

    if (context_ptr->global_mv_injection) {
        /**************
         GLOBALMV L0
        ************* */
        {
            int16_t to_inject_mv_x = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[1] >> GM_TRANS_ONLY_PREC_DIFF);
            int16_t to_inject_mv_y = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[0] >> GM_TRANS_ONLY_PREC_DIFF);
            if (context_ptr->injected_mv_count_l0 == 0 || is_already_injected_mv_l0(context_ptr, to_inject_mv_x, to_inject_mv_y) == EB_FALSE) {
                candidateArray[canTotalCnt].type = INTER_MODE;
                candidateArray[canTotalCnt].distortion_ready = 0;
                candidateArray[canTotalCnt].use_intrabc = 0;
                candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)0;
                candidateArray[canTotalCnt].inter_mode = GLOBALMV;
                candidateArray[canTotalCnt].pred_mode = GLOBALMV;
                candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                candidateArray[canTotalCnt].is_compound = 0;
                candidateArray[canTotalCnt].is_new_mv = 0;
                candidateArray[canTotalCnt].is_zero_mv = 0;
                candidateArray[canTotalCnt].drl_index = 0;

                // will be needed later by the rate estimation
                candidateArray[canTotalCnt].ref_mv_index = 0;
                candidateArray[canTotalCnt].pred_mv_weight = 0;
                candidateArray[canTotalCnt].ref_frame_type = LAST_FRAME;

                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

                // Set the MV to frame MV
                candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x;
                candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y;

                ++canTotalCnt;
                context_ptr->injected_mv_x_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_x;
                context_ptr->injected_mv_y_l0_array[context_ptr->injected_mv_count_l0] = to_inject_mv_y;
                ++context_ptr->injected_mv_count_l0;
            }
        }

        if (isCompoundEnabled && allow_bipred) {
            /**************
            GLOBAL_GLOBALMV
            ************* */
            int16_t to_inject_mv_x_l0 = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[1] >> GM_TRANS_ONLY_PREC_DIFF);
            int16_t to_inject_mv_y_l0 = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[LAST_FRAME].wmmat[0] >> GM_TRANS_ONLY_PREC_DIFF);
            int16_t to_inject_mv_x_l1 = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[BWDREF_FRAME].wmmat[1] >> GM_TRANS_ONLY_PREC_DIFF);
            int16_t to_inject_mv_y_l1 = (int16_t)(picture_control_set_ptr->parent_pcs_ptr->global_motion[BWDREF_FRAME].wmmat[0] >> GM_TRANS_ONLY_PREC_DIFF);
            if (context_ptr->injected_mv_count_bipred == 0 || is_already_injected_mv_bipred(context_ptr, to_inject_mv_x_l0, to_inject_mv_y_l0, to_inject_mv_x_l1, to_inject_mv_y_l1) == EB_FALSE) {
                candidateArray[canTotalCnt].type = INTER_MODE;
                candidateArray[canTotalCnt].distortion_ready = 0;
                candidateArray[canTotalCnt].use_intrabc = 0;
                candidateArray[canTotalCnt].merge_flag = EB_FALSE;
                candidateArray[canTotalCnt].prediction_direction[0] = (EbPredDirection)2;
                candidateArray[canTotalCnt].inter_mode = GLOBAL_GLOBALMV;
                candidateArray[canTotalCnt].pred_mode = GLOBAL_GLOBALMV;
                candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
                candidateArray[canTotalCnt].is_compound = 1;
                candidateArray[canTotalCnt].is_new_mv = 0;
                candidateArray[canTotalCnt].is_zero_mv = 0;
                candidateArray[canTotalCnt].drl_index = 0;

                // will be needed later by the rate estimation
                candidateArray[canTotalCnt].ref_mv_index = 0;
                candidateArray[canTotalCnt].pred_mv_weight = 0;
                candidateArray[canTotalCnt].ref_frame_type = LAST_BWD_FRAME;

                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;

                // Set the MV to frame MV
                candidateArray[canTotalCnt].motion_vector_xl0 = to_inject_mv_x_l0;
                candidateArray[canTotalCnt].motion_vector_yl0 = to_inject_mv_y_l0;
                candidateArray[canTotalCnt].motion_vector_xl1 = to_inject_mv_x_l1;
                candidateArray[canTotalCnt].motion_vector_yl1 = to_inject_mv_y_l1;

                ++canTotalCnt;
                context_ptr->injected_mv_x_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l0;
                context_ptr->injected_mv_y_bipred_l0_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l0;
                context_ptr->injected_mv_x_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_x_l1;
                context_ptr->injected_mv_y_bipred_l1_array[context_ptr->injected_mv_count_bipred] = to_inject_mv_y_l1;
                ++context_ptr->injected_mv_count_bipred;
            }
        }
    }
    // Warped Motion
    if (picture_control_set_ptr->parent_pcs_ptr->allow_warped_motion &&
        has_overlappable_candidates(context_ptr->cu_ptr) &&
        context_ptr->blk_geom->bwidth >= 8 &&
        context_ptr->blk_geom->bheight >= 8 &&
        context_ptr->warped_motion_injection){
        inject_warped_motion_candidates(
            picture_control_set_ptr,
            context_ptr,
            context_ptr->cu_ptr,
            &canTotalCnt,
            ss_mecontext,
            mePuResult,
            use_close_loop_me,
            close_loop_me_index);
    }

    if (inject_newmv_candidate) {
#if BASE_LAYER_REF || MRP_REF_MODE
        if (isCompoundEnabled) {
            if (allow_bipred) {
#else
        if (allow_bipred) {
#endif
            //----------------------
            // Bipred2Nx2N
            //----------------------
            if (context_ptr->bipred3x3_injection > 0)
                if (picture_control_set_ptr->slice_type == B_SLICE)
                    Bipred3x3CandidatesInjection(
                        picture_control_set_ptr,
                        context_ptr,
                        sb_ptr,
                        me_sb_addr,
                        ss_mecontext,
                        use_close_loop_me,
                        close_loop_me_index,
                        me2Nx2NTableOffset,
                        &canTotalCnt);
#if BASE_LAYER_REF || MRP_REF_MODE
            }
#endif
            //----------------------
            // Unipred2Nx2N
            //----------------------
            if (context_ptr->unipred3x3_injection > 0)
                if (picture_control_set_ptr->slice_type != I_SLICE)
                    Unipred3x3CandidatesInjection(
                        picture_control_set_ptr,
                        context_ptr,
                        sb_ptr,
                        me_sb_addr,
                        ss_mecontext,
                        use_close_loop_me,
                        close_loop_me_index,
                        me2Nx2NTableOffset,
                        &canTotalCnt);
        }
    }
    // update the total number of candidates injected
    (*candidateTotalCnt) = canTotalCnt;

    return;
}
#endif

 extern PredictionMode get_uv_mode(UvPredictionMode mode) {
    assert(mode < UV_INTRA_MODES);
    static const PredictionMode uv2y[] = {
        DC_PRED,        // UV_DC_PRED
        V_PRED,         // UV_V_PRED
        H_PRED,         // UV_H_PRED
        D45_PRED,       // UV_D45_PRED
        D135_PRED,      // UV_D135_PRED
        D113_PRED,      // UV_D113_PRED
        D157_PRED,      // UV_D157_PRED
        D203_PRED,      // UV_D203_PRED
        D67_PRED,       // UV_D67_PRED
        SMOOTH_PRED,    // UV_SMOOTH_PRED
        SMOOTH_V_PRED,  // UV_SMOOTH_V_PRED
        SMOOTH_H_PRED,  // UV_SMOOTH_H_PRED
        PAETH_PRED,     // UV_PAETH_PRED
        DC_PRED,        // UV_CFL_PRED
        INTRA_INVALID,  // UV_INTRA_MODES
        INTRA_INVALID,  // UV_MODE_INVALID
    };
    return uv2y[mode];
}
static TxType intra_mode_to_tx_type(const MbModeInfo *mbmi,
    PlaneType plane_type) {
    static const TxType _intra_mode_to_tx_type[INTRA_MODES] = {
        DCT_DCT,    // DC
        ADST_DCT,   // V
        DCT_ADST,   // H
        DCT_DCT,    // D45
        ADST_ADST,  // D135
        ADST_DCT,   // D117
        DCT_ADST,   // D153
        DCT_ADST,   // D207
        ADST_DCT,   // D63
        ADST_ADST,  // SMOOTH
        ADST_DCT,   // SMOOTH_V
        DCT_ADST,   // SMOOTH_H
        ADST_ADST,  // PAETH
    };
    const PredictionMode mode =
        (plane_type == PLANE_TYPE_Y) ? mbmi->mode : get_uv_mode(mbmi->uv_mode);
    assert(mode < INTRA_MODES);
    return _intra_mode_to_tx_type[mode];
}

static INLINE TxType av1_get_tx_type(
    BlockSize  sb_type,
    int32_t   is_inter,
    PredictionMode pred_mode,
    UvPredictionMode pred_mode_uv,
    PlaneType plane_type,
    const MacroBlockD *xd, int32_t blk_row,
    int32_t blk_col, TxSize tx_size,
    int32_t reduced_tx_set)
{
    UNUSED(sb_type);
    UNUSED(*xd);
    UNUSED(blk_row);
    UNUSED(blk_col);

    // BlockSize  sb_type = BLOCK_8X8;

    MbModeInfo  mbmi;
    mbmi.mode = pred_mode;
    mbmi.uv_mode = pred_mode_uv;

    // const MbModeInfo *const mbmi = xd->mi[0];
    // const struct MacroblockdPlane *const pd = &xd->plane[plane_type];
    const TxSetType tx_set_type =
        /*av1_*/get_ext_tx_set_type(tx_size, is_inter, reduced_tx_set);

    TxType tx_type = DCT_DCT;
    if ( /*xd->lossless[mbmi->segment_id] ||*/ txsize_sqr_up_map[tx_size] > TX_32X32)
        tx_type = DCT_DCT;
    else {
        if (plane_type == PLANE_TYPE_Y) {
            //const int32_t txk_type_idx =
            //    av1_get_txk_type_index(/*mbmi->*/sb_type, blk_row, blk_col);
            //tx_type = mbmi->txk_type[txk_type_idx];
        }
        else if (is_inter /*is_inter_block(mbmi)*/) {
            // scale back to y plane's coordinate
            //blk_row <<= pd->subsampling_y;
            //blk_col <<= pd->subsampling_x;
            //const int32_t txk_type_idx =
            //    av1_get_txk_type_index(mbmi->sb_type, blk_row, blk_col);
            //tx_type = mbmi->txk_type[txk_type_idx];
        }
        else {
            // In intra mode, uv planes don't share the same prediction mode as y
            // plane, so the tx_type should not be shared
            tx_type = intra_mode_to_tx_type(&mbmi, PLANE_TYPE_UV);
        }
    }
    ASSERT(tx_type < TX_TYPES);
    if (!av1_ext_tx_used[tx_set_type][tx_type]) return DCT_DCT;
    return tx_type;
}

void  inject_intra_candidates_ois(
    PictureControlSet            *picture_control_set_ptr,
    ModeDecisionContext          *context_ptr,
    LargestCodingUnit            *sb_ptr,
    uint32_t                       *candidate_total_cnt){
    uint8_t                     intra_candidate_counter;
    uint8_t                     intra_mode;
    uint32_t                    can_total_cnt = 0;
    ModeDecisionCandidate    *candidate_array = context_ptr->fast_candidate_array;
#if DISABLE_CFL
    EbBool disable_cfl_flag = 1;
#else
#if CFL_FIX
    EbBool                      disable_cfl_flag = (MAX(context_ptr->blk_geom->bheight, context_ptr->blk_geom->bwidth) > 32) ? EB_TRUE : EB_FALSE;
#else
    EbBool                      disable_cfl_flag = (context_ptr->blk_geom->sq_size > 32 ||
                                                    context_ptr->blk_geom->bwidth == 4  ||
                                                    context_ptr->blk_geom->bheight == 4)    ? EB_TRUE : EB_FALSE;
#endif
#endif
    OisSbResults    *ois_sb_results_ptr = picture_control_set_ptr->parent_pcs_ptr->ois_sb_results[sb_ptr->index];
    OisCandidate     *ois_blk_ptr = ois_sb_results_ptr->ois_candidate_array[ep_to_pa_block_index[context_ptr->blk_geom->blkidx_mds]];
    uint8_t              total_intra_luma_mode = ois_sb_results_ptr-> total_ois_intra_candidate[ep_to_pa_block_index[context_ptr->blk_geom->blkidx_mds]];

    for (intra_candidate_counter = 0; intra_candidate_counter < total_intra_luma_mode; ++intra_candidate_counter) {
        intra_mode = ois_blk_ptr[can_total_cnt].intra_mode;
        assert(intra_mode < INTRA_MODES);
        if (av1_is_directional_mode((PredictionMode)intra_mode)) {
            int32_t angle_delta = ois_blk_ptr[can_total_cnt].angle_delta ;
            candidate_array[can_total_cnt].type = INTRA_MODE;
            candidate_array[can_total_cnt].intra_luma_mode = intra_mode;
            candidate_array[can_total_cnt].distortion_ready =  1;
            candidate_array[can_total_cnt].me_distortion = ois_blk_ptr[can_total_cnt].distortion;
            candidate_array[can_total_cnt].use_intrabc = 0;
            candidate_array[can_total_cnt].is_directional_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)intra_mode);
#if !SEARCH_UV_CLEAN_UP
            candidate_array[can_total_cnt].use_angle_delta = use_angle_delta ? candidate_array[can_total_cnt].is_directional_mode_flag : 0;
#endif
            candidate_array[can_total_cnt].angle_delta[PLANE_TYPE_Y] = angle_delta;
            candidate_array[can_total_cnt].intra_chroma_mode = disable_cfl_flag ? intra_luma_to_chroma[intra_mode] :
                                                               context_ptr->chroma_level <= CHROMA_MODE_1 ? UV_CFL_PRED : UV_DC_PRED;

            candidate_array[can_total_cnt].cfl_alpha_signs = 0;
            candidate_array[can_total_cnt].cfl_alpha_idx = 0;
            candidate_array[can_total_cnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)candidate_array[can_total_cnt].intra_chroma_mode);
            candidate_array[can_total_cnt].angle_delta[PLANE_TYPE_UV] = 0;

#if ATB_TX_TYPE_SUPPORT_PER_TU
            candidate_array[can_total_cnt].transform_type[0] = DCT_DCT;

            if (candidate_array[can_total_cnt].intra_chroma_mode == UV_CFL_PRED)
                candidate_array[can_total_cnt].transform_type_uv = DCT_DCT;
            else
                candidate_array[can_total_cnt].transform_type_uv =
#else
            candidate_array[can_total_cnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;

            if (candidate_array[can_total_cnt].intra_chroma_mode == UV_CFL_PRED)
                candidate_array[can_total_cnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
            else
                candidate_array[can_total_cnt].transform_type[PLANE_TYPE_UV] =
#endif
                av1_get_tx_type(
                    context_ptr->blk_geom->bsize,
                    0,
                    (PredictionMode)candidate_array[can_total_cnt].intra_luma_mode,
                    (UvPredictionMode)candidate_array[can_total_cnt].intra_chroma_mode,
                    PLANE_TYPE_UV,
                    0,
                    0,
                    0,
#if ATB_SUPPORT
                    context_ptr->blk_geom->txsize_uv[0][0],
#else
                    context_ptr->blk_geom->txsize_uv[0],
#endif
                    picture_control_set_ptr->parent_pcs_ptr->reduced_tx_set_used);
            candidate_array[can_total_cnt].ref_frame_type = INTRA_FRAME;
            candidate_array[can_total_cnt].pred_mode = (PredictionMode)intra_mode;
            candidate_array[can_total_cnt].motion_mode = SIMPLE_TRANSLATION;
#if CHECK_CAND
            INCRMENT_CAND_TOTAL_COUNT(can_total_cnt);
#else
            ++can_total_cnt;
#endif
        }
        else {
            candidate_array[can_total_cnt].type = INTRA_MODE;
            candidate_array[can_total_cnt].intra_luma_mode = intra_mode;
            candidate_array[can_total_cnt].distortion_ready =  1;
            candidate_array[can_total_cnt].me_distortion = ois_blk_ptr[can_total_cnt].distortion;
            candidate_array[can_total_cnt].use_intrabc = 0;
            candidate_array[can_total_cnt].is_directional_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)intra_mode);
#if !SEARCH_UV_CLEAN_UP
            candidate_array[can_total_cnt].use_angle_delta = candidate_array[can_total_cnt].is_directional_mode_flag;
#endif
            candidate_array[can_total_cnt].angle_delta[PLANE_TYPE_Y] = 0;
            candidate_array[can_total_cnt].intra_chroma_mode =  disable_cfl_flag ? intra_luma_to_chroma[intra_mode] :
                                                                context_ptr->chroma_level <= CHROMA_MODE_1 ? UV_CFL_PRED : UV_DC_PRED;

            candidate_array[can_total_cnt].cfl_alpha_signs = 0;
            candidate_array[can_total_cnt].cfl_alpha_idx = 0;
            candidate_array[can_total_cnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)candidate_array[can_total_cnt].intra_chroma_mode);
            candidate_array[can_total_cnt].angle_delta[PLANE_TYPE_UV] = 0;
#if ATB_TX_TYPE_SUPPORT_PER_TU
            candidate_array[can_total_cnt].transform_type[0] = DCT_DCT;

            if (candidate_array[can_total_cnt].intra_chroma_mode == UV_CFL_PRED)
                candidate_array[can_total_cnt].transform_type_uv = DCT_DCT;
            else
                candidate_array[can_total_cnt].transform_type_uv =
#else
            candidate_array[can_total_cnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;

            if (candidate_array[can_total_cnt].intra_chroma_mode == UV_CFL_PRED)
                candidate_array[can_total_cnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
            else
                candidate_array[can_total_cnt].transform_type[PLANE_TYPE_UV] =
#endif
                av1_get_tx_type(
                    context_ptr->blk_geom->bsize,
                    0,
                    (PredictionMode)candidate_array[can_total_cnt].intra_luma_mode,
                    (UvPredictionMode)candidate_array[can_total_cnt].intra_chroma_mode,
                    PLANE_TYPE_UV,
                    0,
                    0,
                    0,
#if ATB_SUPPORT
                    context_ptr->blk_geom->txsize_uv[0][0],
#else
                    context_ptr->blk_geom->txsize_uv[0],
#endif
                    picture_control_set_ptr->parent_pcs_ptr->reduced_tx_set_used);
            candidate_array[can_total_cnt].ref_frame_type = INTRA_FRAME;
            candidate_array[can_total_cnt].pred_mode = (PredictionMode)intra_mode;
            candidate_array[can_total_cnt].motion_mode = SIMPLE_TRANSLATION;
#if CHECK_CAND
            INCRMENT_CAND_TOTAL_COUNT(can_total_cnt);
#else
            ++can_total_cnt;
#endif
        }
    }

    // update the total number of candidates injected
    (*candidate_total_cnt) = can_total_cnt;

    return;
}

double av1_convert_qindex_to_q(int32_t qindex, AomBitDepth bit_depth);

static INLINE void setup_pred_plane(struct Buf2D *dst, BlockSize bsize,
    uint8_t *src, int width, int height,
    int stride, int mi_row, int mi_col,
    int subsampling_x, int subsampling_y) {
    // Offset the buffer pointer
    if (subsampling_y && (mi_row & 0x01) && (mi_size_high[bsize] == 1))
        mi_row -= 1;
    if (subsampling_x && (mi_col & 0x01) && (mi_size_wide[bsize] == 1))
        mi_col -= 1;

    const int x = (MI_SIZE * mi_col) >> subsampling_x;
    const int y = (MI_SIZE * mi_row) >> subsampling_y;
    dst->buf = src + (y * stride + x);// scaled_buffer_offset(x, y, stride, scale);
    dst->buf0 = src;
    dst->width = width;
    dst->height = height;
    dst->stride = stride;
}
void av1_setup_pred_block(BlockSize sb_type,
    struct Buf2D dst[MAX_MB_PLANE],
    const Yv12BufferConfig *src, int mi_row, int mi_col) {
    int i;

    dst[0].buf = src->y_buffer;
    dst[0].stride = src->y_stride;
    dst[1].buf = src->u_buffer;
    dst[2].buf = src->v_buffer;
    dst[1].stride = dst[2].stride = src->uv_stride;

    i = 0;
    setup_pred_plane(dst + i, sb_type, dst[i].buf,
        i ? src->uv_crop_width : src->y_crop_width,
        i ? src->uv_crop_height : src->y_crop_height,
        dst[i].stride, mi_row, mi_col,
        0, 0);
}
// Values are now correlated to quantizer.
static int sad_per_bit16lut_8[QINDEX_RANGE];
static int sad_per_bit4lut_8[QINDEX_RANGE];

static void init_me_luts_bd(int *bit16lut, int *bit4lut, int range,
    AomBitDepth bit_depth) {
    int i;
    // Initialize the sad lut tables using a formulaic calculation for now.
    // This is to make it easier to resolve the impact of experimental changes
    // to the quantizer tables.
    for (i = 0; i < range; i++) {
        const double q = av1_convert_qindex_to_q(i, bit_depth);
        bit16lut[i] = (int)(0.0418 * q + 2.4107);
        bit4lut[i] = (int)(0.063 * q + 2.742);
    }
}

void av1_init_me_luts(void) {
    init_me_luts_bd(sad_per_bit16lut_8, sad_per_bit4lut_8, QINDEX_RANGE,
        AOM_BITS_8);
}

static INLINE int mv_check_bounds(const MvLimits *mv_limits, const MV *mv) {
    return (mv->row >> 3) < mv_limits->row_min ||
        (mv->row >> 3) > mv_limits->row_max ||
        (mv->col >> 3) < mv_limits->col_min ||
        (mv->col >> 3) > mv_limits->col_max;
}
void assert_release(int statement)
{
    if (statement == 0)
        printf("ASSERT_ERRRR\n");
}

void  intra_bc_search(
    PictureControlSet            *pcs,
    ModeDecisionContext          *context_ptr,
    const SequenceControlSet     *scs,
#if !PREDICTIVE_ME
    LargestCodingUnit            *sb_ptr,
#endif
    CodingUnit                   *cu_ptr,
    MV                             *dv_cand,
    uint8_t                        *num_dv_cand)
{
    IntraBcContext  x_st;
    IntraBcContext  *x = &x_st;
    //fill x with what needed.
    x->is_exhaustive_allowed =  context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4 ? 1 : 0;
    //CHKN crc calculator could be moved to mdContext and these init at init time.
    av1_crc_calculator_init(&x->crc_calculator1, 24, 0x5D6DCB);
    av1_crc_calculator_init(&x->crc_calculator2, 24, 0x864CFB);

    x->xd = cu_ptr->av1xd;
    x->nmv_vec_cost = context_ptr->md_rate_estimation_ptr->nmv_vec_cost;
    x->mv_cost_stack = context_ptr->md_rate_estimation_ptr->nmvcoststack;
    BlockSize bsize = context_ptr->blk_geom->bsize;
    assert(bsize < BlockSizeS_ALL);
    const Av1Common *const cm = pcs->parent_pcs_ptr->av1_cm;
    MvReferenceFrame ref_frame = INTRA_FRAME;
#if !PREDICTIVE_ME
    generate_av1_mvp_table(
        &sb_ptr->tile_info,
        context_ptr,
        context_ptr->cu_ptr,
        context_ptr->blk_geom,
        context_ptr->cu_origin_x,
        context_ptr->cu_origin_y,
        &ref_frame,
        1,
        pcs);
#endif
    const int num_planes = 3;
    MacroBlockD * xd = cu_ptr->av1xd;
    const TileInfo *tile = &xd->tile;
    const int mi_row = -xd->mb_to_top_edge / (8 * MI_SIZE);
    const int mi_col = -xd->mb_to_left_edge / (8 * MI_SIZE);
    const int w = block_size_wide[bsize];
    const int h = block_size_high[bsize];
    const int sb_row = mi_row >> scs->seq_header.sb_size_log2;
    const int sb_col = mi_col >> scs->seq_header.sb_size_log2;

    // Set up limit values for MV components.
    // Mv beyond the range do not produce new/different prediction block.
    const int mi_width = mi_size_wide[bsize];
    const int mi_height = mi_size_high[bsize];
    x->mv_limits.row_min =
        -(((mi_row + mi_height) * MI_SIZE) + AOM_INTERP_EXTEND);
    x->mv_limits.col_min = -(((mi_col + mi_width) * MI_SIZE) + AOM_INTERP_EXTEND);
    x->mv_limits.row_max = (cm->mi_rows - mi_row) * MI_SIZE + AOM_INTERP_EXTEND;
    x->mv_limits.col_max = (cm->mi_cols - mi_col) * MI_SIZE + AOM_INTERP_EXTEND;
    //set search paramters
    x->sadperbit16 = sad_per_bit16lut_8[pcs->parent_pcs_ptr->base_qindex];
    x->errorperbit = context_ptr->full_lambda >> RD_EPB_SHIFT;
    x->errorperbit += (x->errorperbit == 0);
    //temp buffer for hash me
    for (int xi = 0; xi < 2; xi++)
        for (int yj = 0; yj < 2; yj++)
            x->hash_value_buffer[xi][yj] = (uint32_t*)malloc(AOM_BUFFER_SIZE_FOR_BLOCK_HASH * sizeof(uint32_t));

    IntMv nearestmv, nearmv;
    av1_find_best_ref_mvs_from_stack(0, context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack /*mbmi_ext*/, xd, ref_frame, &nearestmv, &nearmv,
        0);
    if (nearestmv.as_int == INVALID_MV)
        nearestmv.as_int = 0;
    if (nearmv.as_int == INVALID_MV)
        nearmv.as_int = 0;
    IntMv dv_ref = nearestmv.as_int == 0 ? nearmv : nearestmv;
    if (dv_ref.as_int == 0)
        av1_find_ref_dv(&dv_ref, tile, scs->seq_header.sb_mi_size, mi_row, mi_col);
    // Ref DV should not have sub-pel.
    assert((dv_ref.as_mv.col & 7) == 0);
    assert((dv_ref.as_mv.row & 7) == 0);
    context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[INTRA_FRAME][0].this_mv = dv_ref;

    /* pointer to current frame */
    Yv12BufferConfig cur_buf;
    link_Eb_to_aom_buffer_desc_8bit(
        pcs->parent_pcs_ptr->enhanced_picture_ptr,
        &cur_buf);
    struct Buf2D yv12_mb[MAX_MB_PLANE];
    av1_setup_pred_block(bsize, yv12_mb, &cur_buf, mi_row, mi_col);
    for (int i = 0; i < num_planes; ++i)
        x->xdplane[i].pre[0] = yv12_mb[i];  //ref in ME
    //setup src for DV search same as ref
    x->plane[0].src = x->xdplane[0].pre[0];

    enum IntrabcMotionDirection {
        IBC_MOTION_ABOVE,
        IBC_MOTION_LEFT,
        IBC_MOTION_DIRECTIONS
    };

    //up to two dv candidates will be generated
    enum IntrabcMotionDirection max_dir = pcs->parent_pcs_ptr->ibc_mode > 1 ? IBC_MOTION_LEFT : IBC_MOTION_DIRECTIONS;

    for (enum IntrabcMotionDirection dir = IBC_MOTION_ABOVE;
        dir < max_dir; ++dir) {
        const MvLimits tmp_mv_limits = x->mv_limits;

        switch (dir) {
        case IBC_MOTION_ABOVE:
            x->mv_limits.col_min = (tile->mi_col_start - mi_col) * MI_SIZE;
            x->mv_limits.col_max = (tile->mi_col_end - mi_col) * MI_SIZE - w;
            x->mv_limits.row_min = (tile->mi_row_start - mi_row) * MI_SIZE;
            x->mv_limits.row_max =
                (sb_row * scs->seq_header.sb_mi_size - mi_row) * MI_SIZE - h;
            break;
        case IBC_MOTION_LEFT:
            x->mv_limits.col_min = (tile->mi_col_start - mi_col) * MI_SIZE;
            x->mv_limits.col_max =
                (sb_col * scs->seq_header.sb_mi_size - mi_col) * MI_SIZE - w;
            // TODO: Minimize the overlap between above and
            // left areas.
            x->mv_limits.row_min = (tile->mi_row_start - mi_row) * MI_SIZE;
            int bottom_coded_mi_edge =
                AOMMIN((sb_row + 1) * scs->seq_header.sb_mi_size, tile->mi_row_end);
            x->mv_limits.row_max = (bottom_coded_mi_edge - mi_row) * MI_SIZE - h;
            break;
        default: assert(0);
        }
        assert_release(x->mv_limits.col_min >= tmp_mv_limits.col_min);
        assert_release(x->mv_limits.col_max <= tmp_mv_limits.col_max);
        assert_release(x->mv_limits.row_min >= tmp_mv_limits.row_min);
        assert_release(x->mv_limits.row_max <= tmp_mv_limits.row_max);

        av1_set_mv_search_range(&x->mv_limits, &dv_ref.as_mv);

        if (x->mv_limits.col_max < x->mv_limits.col_min ||
            x->mv_limits.row_max < x->mv_limits.row_min) {
            x->mv_limits = tmp_mv_limits;
            continue;
        }

        int step_param = 0;
        MV mvp_full = dv_ref.as_mv;
        mvp_full.col >>= 3;
        mvp_full.row >>= 3;
        const int sadpb = x->sadperbit16;
        x->best_mv.as_int = 0;

#define INT_VAR_MAX  2147483647    // maximum (signed) int value

        const int bestsme = av1_full_pixel_search(
            pcs, x, bsize, &mvp_full, step_param, 1, 0,
            sadpb, NULL, &dv_ref.as_mv, INT_VAR_MAX, 1,
            (MI_SIZE * mi_col), (MI_SIZE * mi_row), 1);

        x->mv_limits = tmp_mv_limits;
        if (bestsme == INT_VAR_MAX) continue;
        mvp_full = x->best_mv.as_mv;

        const MV dv = { .row = mvp_full.row * 8,.col = mvp_full.col * 8 };
        if (mv_check_bounds(&x->mv_limits, &dv)) continue;
        if (!av1_is_dv_valid(dv, xd, mi_row, mi_col, bsize,
            scs->seq_header.sb_size_log2))
            continue;

        // DV should not have sub-pel.
        assert_release((dv.col & 7) == 0);
        assert_release((dv.row & 7) == 0);

        //store output
        dv_cand[*num_dv_cand] = dv;
        (*num_dv_cand)++;
    }

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            free(x->hash_value_buffer[i][j]);
}

void  inject_intra_bc_candidates(
    PictureControlSet            *picture_control_set_ptr,
    ModeDecisionContext          *context_ptr,
    const SequenceControlSet     *sequence_control_set_ptr,
    //LargestCodingUnit            *sb_ptr,
    CodingUnit                   *cu_ptr,
    uint32_t                       *cand_cnt)
{
    MV dv_cand[2];
    uint8_t num_dv_cand = 0;

    //perform dv-pred + search up to 2 dv(s)
    intra_bc_search(
        picture_control_set_ptr,
        context_ptr,
        sequence_control_set_ptr,
#if !PREDICTIVE_ME
        sb_ptr,
#endif
        cu_ptr,
        dv_cand,
        &num_dv_cand);

    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
    uint32_t dv_i;

    for (dv_i = 0; dv_i < num_dv_cand; dv_i++)
    {
        candidateArray[*cand_cnt].type = INTRA_MODE;
        candidateArray[*cand_cnt].intra_luma_mode = DC_PRED;
        candidateArray[*cand_cnt].distortion_ready = 0;
        candidateArray[*cand_cnt].use_intrabc = 1;
        candidateArray[*cand_cnt].is_directional_mode_flag = 0;
#if !SEARCH_UV_CLEAN_UP
        candidateArray[*cand_cnt].use_angle_delta = 0;
#endif
        candidateArray[*cand_cnt].angle_delta[PLANE_TYPE_Y] = 0;
        candidateArray[*cand_cnt].intra_chroma_mode = UV_DC_PRED;
        candidateArray[*cand_cnt].cfl_alpha_signs = 0;
        candidateArray[*cand_cnt].cfl_alpha_idx = 0;
        candidateArray[*cand_cnt].is_directional_chroma_mode_flag = 0;
        candidateArray[*cand_cnt].angle_delta[PLANE_TYPE_UV] = 0;
#if ATB_TX_TYPE_SUPPORT_PER_TU
        candidateArray[*cand_cnt].transform_type[0] = DCT_DCT;
        candidateArray[*cand_cnt].transform_type_uv = DCT_DCT;
#else
        candidateArray[*cand_cnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;
        candidateArray[*cand_cnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
#endif
        candidateArray[*cand_cnt].ref_frame_type = INTRA_FRAME;
        candidateArray[*cand_cnt].pred_mode = DC_PRED;
        candidateArray[*cand_cnt].motion_mode = SIMPLE_TRANSLATION;
        //inter ralated
        candidateArray[*cand_cnt].is_compound = 0;
        candidateArray[*cand_cnt].merge_flag = EB_FALSE;
        candidateArray[*cand_cnt].prediction_direction[0] = UNI_PRED_LIST_0;
        candidateArray[*cand_cnt].is_new_mv = 0;
        candidateArray[*cand_cnt].is_zero_mv = 0;
        candidateArray[*cand_cnt].motion_vector_xl0 = dv_cand[dv_i].col;
        candidateArray[*cand_cnt].motion_vector_yl0 = dv_cand[dv_i].row;
        candidateArray[*cand_cnt].motion_vector_pred_x[REF_LIST_0] = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[INTRA_FRAME][0].this_mv.as_mv.col;
        candidateArray[*cand_cnt].motion_vector_pred_y[REF_LIST_0] = context_ptr->md_local_cu_unit[context_ptr->blk_geom->blkidx_mds].ed_ref_mv_stack[INTRA_FRAME][0].this_mv.as_mv.row;
        candidateArray[*cand_cnt].drl_index = 0;
        candidateArray[*cand_cnt].ref_mv_index = 0;
        candidateArray[*cand_cnt].pred_mv_weight = 0;
        candidateArray[*cand_cnt].interp_filters = av1_broadcast_interp_filter(BILINEAR);

#if FI_MD
        candidateArray[*cand_cnt].filter_intra_mode = FILTER_INTRA_MODES;
#endif
#if II_COMP
        candidateArray[*cand_cnt].is_interintra_used = 0;
#endif
#if CHECK_CAND
        INCRMENT_CAND_TOTAL_COUNT( (*cand_cnt) );
#else
        ++(*cand_cnt);
#endif
    }
}
#if ESTIMATE_INTRA
// Indices are sign, integer, and fractional part of the gradient value
static const uint8_t gradient_to_angle_bin[2][7][16] = {
  {
      { 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1 },
      { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
      { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
      { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
      { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 },
      { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 },
  },
  {
      { 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4 },
      { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3 },
      { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3 },
      { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3 },
      { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3 },
      { 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2 },
      { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 },
  },
};

/* clang-format off */
static const uint8_t mode_to_angle_bin[INTRA_MODES] = {
  0, 2, 6, 0, 4, 3, 5, 7, 1, 0,
  0,
};
void av1_get_gradient_hist_c(const uint8_t *src, int src_stride, int rows,
    int cols, uint64_t *hist) {
    src += src_stride;
    for (int r = 1; r < rows; ++r) {
        for (int c = 1; c < cols; ++c) {
            int dx = src[c] - src[c - 1];
            int dy = src[c] - src[c - src_stride];
            int index;
            const int temp = dx * dx + dy * dy;
            if (dy == 0) {
                index = 2;
            }
            else {
                const int sn = (dx > 0) ^ (dy > 0);
                dx = abs(dx);
                dy = abs(dy);
                const int remd = (dx % dy) * 16 / dy;
                const int quot = dx / dy;
                index = gradient_to_angle_bin[sn][AOMMIN(quot, 6)][AOMMIN(remd, 15)];
            }
            hist[index] += temp;
        }
        src += src_stride;
    }
}
static void angle_estimation(
    const uint8_t *src,
    int src_stride,
    int rows,
    int cols,
    //BLOCK_SIZE bsize,
    uint8_t *directional_mode_skip_mask)
{
    // Check if angle_delta is used
    //if (!av1_use_angle_delta(bsize)) return;

    uint64_t hist[DIRECTIONAL_MODES] = { 0 };
    //if (is_hbd)
    //    get_highbd_gradient_hist(src, src_stride, rows, cols, hist);
    //else
        av1_get_gradient_hist(src, src_stride, rows, cols, hist);

    int i;
    uint64_t hist_sum = 0;
    for (i = 0; i < DIRECTIONAL_MODES; ++i) hist_sum += hist[i];
    for (i = 0; i < INTRA_MODES; ++i) {
        if (av1_is_directional_mode(i)) {
            const uint8_t angle_bin = mode_to_angle_bin[i];
            uint64_t score = 2 * hist[angle_bin];
            int weight = 2;
            if (angle_bin > 0) {
                score += hist[angle_bin - 1];
                ++weight;
            }
            if (angle_bin < DIRECTIONAL_MODES - 1) {
                score += hist[angle_bin + 1];
                ++weight;
            }
            const int thresh = 10;
            if (score * thresh < hist_sum * weight) directional_mode_skip_mask[i] = 1;
        }
    }
}
#endif
// END of Function Declarations
void  inject_intra_candidates(
    PictureControlSet            *picture_control_set_ptr,
    ModeDecisionContext          *context_ptr,
    const SequenceControlSet     *sequence_control_set_ptr,
    LargestCodingUnit            *sb_ptr,
#if M8_SKIP_BLK
    uint32_t                       *candidateTotalCnt){
#else
    uint32_t                       *candidateTotalCnt,
    const uint32_t                  leaf_index){
    (void)leaf_index;
#endif
    (void)sequence_control_set_ptr;
    (void)sb_ptr;
    uint8_t                     is16bit = (sequence_control_set_ptr->static_config.encoder_bit_depth > EB_8BIT);
    uint8_t                     intra_mode_start = DC_PRED;
    uint8_t                     intra_mode_end   = is16bit ? SMOOTH_H_PRED : PAETH_PRED;
    uint8_t                     openLoopIntraCandidate;
    uint32_t                    canTotalCnt = 0;
    uint8_t                     angleDeltaCounter = 0;
    EbBool                      use_angle_delta = (context_ptr->blk_geom->bsize >= BLOCK_8X8);
    uint8_t                     angleDeltaCandidateCount = use_angle_delta ? 7 : 1;
    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
#if DISABLE_INTRA_4xN_NX4_IN_INTER_PIC
    if ((context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4)  && picture_control_set_ptr->slice_type != I_SLICE) return 0;
#endif
#if DISABLE_CFL
    EbBool disable_cfl_flag = 1;
#else
#if CFL_FIX
    EbBool                      disable_cfl_flag = (MAX(context_ptr->blk_geom->bheight, context_ptr->blk_geom->bwidth) > 32) ? EB_TRUE : EB_FALSE;
#else
    EbBool                      disable_cfl_flag = (context_ptr->blk_geom->sq_size > 32 ||
                                                    context_ptr->blk_geom->bwidth == 4  ||
                                                    context_ptr->blk_geom->bheight == 4)    ? EB_TRUE : EB_FALSE;
#endif
#endif
    uint8_t                     disable_z2_prediction;
    uint8_t                     disable_angle_refinement;
    uint8_t                     disable_angle_prediction;

#if ESTIMATE_INTRA
#if M0_3_CANDIDATE
#if M1_ESTIMATE_INTRA_BASE
#if enable_estimate_angle_intra
    context_ptr->estimate_angle_intra = 1;
#else
    context_ptr->estimate_angle_intra = ((picture_control_set_ptr->enc_mode > ENC_M3) || (picture_control_set_ptr->enc_mode <= ENC_M0 && picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0)) ? 0 : 1;
#endif
#else
    context_ptr->estimate_angle_intra = ((picture_control_set_ptr->enc_mode > ENC_M3) || (picture_control_set_ptr->enc_mode == ENC_M0)) ? 0 : 1;
#endif
#else
    context_ptr->estimate_angle_intra = picture_control_set_ptr->enc_mode <= ENC_M3 && !MR_MODE ? 1 : 0;
#endif

    uint8_t directional_mode_skip_mask[INTRA_MODES] = { 0 };

    if (context_ptr->estimate_angle_intra==1 && use_angle_delta  )
    {
        EbPictureBufferDesc   *src_pic = picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr;
        uint8_t               *src_buf = src_pic->buffer_y + (context_ptr->cu_origin_x + src_pic->origin_x) + (context_ptr->cu_origin_y + src_pic->origin_y) * src_pic->stride_y;
        const int rows = block_size_high[context_ptr->blk_geom->bsize];
        const int cols = block_size_wide[context_ptr->blk_geom->bsize];
        angle_estimation(src_buf, src_pic->stride_y, rows, cols, /*context_ptr->blk_geom->bsize,*/directional_mode_skip_mask);
    }
#endif

#if M9_INTRA
        uint8_t     angle_delta_shift = 1;
    if (picture_control_set_ptr->parent_pcs_ptr->intra_pred_mode == 4) {
        if (picture_control_set_ptr->slice_type == I_SLICE) {
            intra_mode_end = is16bit ? SMOOTH_H_PRED : PAETH_PRED;
#if M10_INTRA
            angleDeltaCandidateCount = use_angle_delta ? 3 : 1;
#else
            angleDeltaCandidateCount = use_angle_delta ? 5 : 1;
#endif
            disable_angle_prediction = 0;
            angle_delta_shift = 2;
            disable_z2_prediction = 0;
        }
        else {
            intra_mode_end = DC_PRED;
            disable_angle_prediction = 1;
            angleDeltaCandidateCount = 1;
            angle_delta_shift = 1;
            disable_z2_prediction = 0;
        }
    }else
#endif
    if (picture_control_set_ptr->parent_pcs_ptr->intra_pred_mode == 3){
        disable_z2_prediction       = 0;
        disable_angle_refinement    = 0;
        disable_angle_prediction    = 1;
#if M9_INTRA
        angleDeltaCandidateCount = disable_angle_refinement ? 1: angleDeltaCandidateCount;
#endif
    } else if (picture_control_set_ptr->parent_pcs_ptr->intra_pred_mode == 2) {
        disable_z2_prediction       = 0;
        disable_angle_refinement    = 0 ;
        disable_angle_prediction    = (context_ptr->blk_geom->sq_size > 16 ||
                                       context_ptr->blk_geom->bwidth == 4 ||
                                       context_ptr->blk_geom->bheight == 4) ? 1 : 0;
#if M9_INTRA
        angleDeltaCandidateCount = disable_angle_refinement ? 1: angleDeltaCandidateCount;
#endif
    } else if (picture_control_set_ptr->parent_pcs_ptr->intra_pred_mode == 1) {
        disable_z2_prediction       = (context_ptr->blk_geom->sq_size > 16 ||
                                       context_ptr->blk_geom->bwidth == 4 ||
                                       context_ptr->blk_geom->bheight == 4) ? 1 : 0;
        disable_angle_refinement    = (context_ptr->blk_geom->sq_size > 16 ||
                                       context_ptr->blk_geom->bwidth == 4 ||
                                       context_ptr->blk_geom->bheight == 4) ? 1 : 0;
        disable_angle_prediction    = 0;
#if M9_INTRA
        angleDeltaCandidateCount = disable_angle_refinement ? 1: angleDeltaCandidateCount;
#endif
    } else {
        disable_z2_prediction       = 0;
        disable_angle_refinement    = 0;
        disable_angle_prediction    = 0;
#if M9_INTRA
        angleDeltaCandidateCount = disable_angle_refinement ? 1: angleDeltaCandidateCount;
#endif
    }
#if MR_MODE
    disable_z2_prediction       = 0;
    disable_angle_refinement    = 0;
    disable_angle_prediction    = 0;
#endif
#if !M9_INTRA
    angleDeltaCandidateCount = disable_angle_refinement ? 1: angleDeltaCandidateCount;
#endif

    for (openLoopIntraCandidate = intra_mode_start; openLoopIntraCandidate <= intra_mode_end ; ++openLoopIntraCandidate) {

        if (av1_is_directional_mode((PredictionMode)openLoopIntraCandidate)) {

#if ESTIMATE_INTRA
            if (!disable_angle_prediction &&
                 directional_mode_skip_mask[(PredictionMode)openLoopIntraCandidate]==0) {
#else
            if (!disable_angle_prediction) {
#endif
                for (angleDeltaCounter = 0; angleDeltaCounter < angleDeltaCandidateCount; ++angleDeltaCounter) {
#if M9_INTRA
                    int32_t angle_delta = CLIP( angle_delta_shift * (angleDeltaCandidateCount == 1 ? 0 : angleDeltaCounter - (angleDeltaCandidateCount >> 1)), -3 , 3);
#else
                    int32_t angle_delta = angleDeltaCandidateCount == 1 ? 0 : angleDeltaCounter - (angleDeltaCandidateCount >> 1);
#endif
                    int32_t  p_angle = mode_to_angle_map[(PredictionMode)openLoopIntraCandidate] + angle_delta * ANGLE_STEP;
                    if (!disable_z2_prediction || (p_angle <= 90 || p_angle >= 180)) {
                        candidateArray[canTotalCnt].type = INTRA_MODE;
                        candidateArray[canTotalCnt].intra_luma_mode = openLoopIntraCandidate;
                        candidateArray[canTotalCnt].distortion_ready = 0;
                        candidateArray[canTotalCnt].use_intrabc = 0;
#if FI_MD
                        candidateArray[canTotalCnt].filter_intra_mode = FILTER_INTRA_MODES;
#endif
                        candidateArray[canTotalCnt].is_directional_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)openLoopIntraCandidate);
#if !SEARCH_UV_CLEAN_UP
                        candidateArray[canTotalCnt].use_angle_delta = use_angle_delta ? candidateArray[canTotalCnt].is_directional_mode_flag : 0;
#endif
                        candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y] = angle_delta;
#if SEARCH_UV_MODE
                        // Search the best independent intra chroma mode
                        if (context_ptr->chroma_level == CHROMA_MODE_0) {
                            candidateArray[canTotalCnt].intra_chroma_mode = disable_cfl_flag ?
                                context_ptr->best_uv_mode[openLoopIntraCandidate][MAX_ANGLE_DELTA + candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y]] :
                                UV_CFL_PRED ;
#if CHROMA_SEARCH_FIX
                            candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = disable_cfl_flag ?
                                context_ptr->best_uv_angle[candidateArray[canTotalCnt].intra_luma_mode][MAX_ANGLE_DELTA + candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y]] : 0;
                            candidateArray[canTotalCnt].is_directional_chroma_mode_flag = disable_cfl_flag ?
                                (uint8_t)av1_is_directional_mode((PredictionMode)(context_ptr->best_uv_mode[candidateArray[canTotalCnt].intra_luma_mode][MAX_ANGLE_DELTA + candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y]])) : 0;
#endif
                        }
                        else {
                            // Hsan/Omar: why the restriction below ? (i.e. disable_ang_uv)
                            const int32_t disable_ang_uv = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) && context_ptr->blk_geom->has_uv ? 1 : 0;
                            candidateArray[canTotalCnt].intra_chroma_mode = disable_cfl_flag ?
                                intra_luma_to_chroma[openLoopIntraCandidate] :
                                (context_ptr->chroma_level == CHROMA_MODE_1) ?
                                UV_CFL_PRED :
                                UV_DC_PRED;
                            candidateArray[canTotalCnt].intra_chroma_mode = disable_ang_uv && av1_is_directional_mode(candidateArray[canTotalCnt].intra_chroma_mode) ?
                                UV_DC_PRED : candidateArray[canTotalCnt].intra_chroma_mode;
#if CHROMA_SEARCH_FIX
                            candidateArray[canTotalCnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)candidateArray[canTotalCnt].intra_chroma_mode);
                            candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = 0;
#endif
                        }
#else
                        const int32_t disable_ang_uv = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) && context_ptr->blk_geom->has_uv ? 1 : 0;
                        candidateArray[canTotalCnt].intra_chroma_mode = disable_cfl_flag ?
                            intra_luma_to_chroma[openLoopIntraCandidate] :
                            (context_ptr->chroma_level <= CHROMA_MODE_1) ?
                                UV_CFL_PRED :
                                UV_DC_PRED;
                        candidateArray[canTotalCnt].intra_chroma_mode = disable_ang_uv && av1_is_directional_mode(candidateArray[canTotalCnt].intra_chroma_mode) ?
                            UV_DC_PRED : candidateArray[canTotalCnt].intra_chroma_mode;
#endif
#if CHROMA_DC_ONLY
                        candidateArray[canTotalCnt].intra_chroma_mode = UV_DC_PRED;
#if CHROMA_SEARCH_FIX
                        candidateArray[canTotalCnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)candidateArray[canTotalCnt].intra_chroma_mode);
                        candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = 0;
#endif
#endif
                        candidateArray[canTotalCnt].cfl_alpha_signs = 0;
                        candidateArray[canTotalCnt].cfl_alpha_idx = 0;
#if !CHROMA_SEARCH_FIX
                        candidateArray[canTotalCnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)candidateArray[canTotalCnt].intra_chroma_mode);
                        candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = 0;
#endif
#if ATB_TX_TYPE_SUPPORT_PER_TU
                        candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;

                        if (candidateArray[canTotalCnt].intra_chroma_mode == UV_CFL_PRED)
                            candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
                        else
                            candidateArray[canTotalCnt].transform_type_uv =
#else
                        candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;

                        if (candidateArray[canTotalCnt].intra_chroma_mode == UV_CFL_PRED)
                            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
                        else
                            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] =
#endif
                            av1_get_tx_type(
                                context_ptr->blk_geom->bsize,
                                0,
                                (PredictionMode)candidateArray[canTotalCnt].intra_luma_mode,
                                (UvPredictionMode)candidateArray[canTotalCnt].intra_chroma_mode,
                                PLANE_TYPE_UV,
                                0,
                                0,
                                0,
#if ATB_SUPPORT
                                context_ptr->blk_geom->txsize_uv[0][0],
#else
                                context_ptr->blk_geom->txsize_uv[0],
#endif
                                picture_control_set_ptr->parent_pcs_ptr->reduced_tx_set_used);
                        candidateArray[canTotalCnt].ref_frame_type = INTRA_FRAME;
                        candidateArray[canTotalCnt].pred_mode = (PredictionMode)openLoopIntraCandidate;
                        candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
#if CHECK_CAND
                        INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
                        ++canTotalCnt;
#endif
                    }
            }
        }
        }
        else {
            candidateArray[canTotalCnt].type = INTRA_MODE;
            candidateArray[canTotalCnt].intra_luma_mode = openLoopIntraCandidate;
            candidateArray[canTotalCnt].distortion_ready = 0;
            candidateArray[canTotalCnt].use_intrabc = 0;
#if FI_MD
            candidateArray[canTotalCnt].filter_intra_mode = FILTER_INTRA_MODES;
#endif
            candidateArray[canTotalCnt].is_directional_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)openLoopIntraCandidate);
#if !SEARCH_UV_CLEAN_UP
            candidateArray[canTotalCnt].use_angle_delta = candidateArray[canTotalCnt].is_directional_mode_flag;
#endif
            candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y] = 0;
#if SEARCH_UV_MODE
            // Search the best independent intra chroma mode
            if (context_ptr->chroma_level == CHROMA_MODE_0) {
                candidateArray[canTotalCnt].intra_chroma_mode = disable_cfl_flag ?
                    context_ptr->best_uv_mode[openLoopIntraCandidate][MAX_ANGLE_DELTA + candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y]] :
                    UV_CFL_PRED;
#if CHROMA_SEARCH_FIX
                candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = disable_cfl_flag ?
                    context_ptr->best_uv_angle[candidateArray[canTotalCnt].intra_luma_mode][MAX_ANGLE_DELTA + candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y]] : 0;
                candidateArray[canTotalCnt].is_directional_chroma_mode_flag = disable_cfl_flag ?
                    (uint8_t)av1_is_directional_mode((PredictionMode)(context_ptr->best_uv_mode[candidateArray[canTotalCnt].intra_luma_mode][MAX_ANGLE_DELTA + candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y]])) : 0;
#endif
            }
            else {
                // Hsan/Omar: why the restriction below ? (i.e. disable_ang_uv)
                const int32_t disable_ang_uv = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) && context_ptr->blk_geom->has_uv ? 1 : 0;
                candidateArray[canTotalCnt].intra_chroma_mode = disable_cfl_flag ?
                    intra_luma_to_chroma[openLoopIntraCandidate] :
                    (context_ptr->chroma_level == CHROMA_MODE_1) ?
                        UV_CFL_PRED :
                        UV_DC_PRED;

                candidateArray[canTotalCnt].intra_chroma_mode = disable_ang_uv && av1_is_directional_mode(candidateArray[canTotalCnt].intra_chroma_mode) ?
                    UV_DC_PRED : candidateArray[canTotalCnt].intra_chroma_mode;

#if CHROMA_SEARCH_FIX
                candidateArray[canTotalCnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)candidateArray[canTotalCnt].intra_chroma_mode);
                candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = 0;
#endif

            }
#else
            const int32_t disable_ang_uv = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) && context_ptr->blk_geom->has_uv ? 1 : 0;
            candidateArray[canTotalCnt].intra_chroma_mode = disable_cfl_flag ?
                intra_luma_to_chroma[openLoopIntraCandidate] :
                (context_ptr->chroma_level <= CHROMA_MODE_1) ?
                    UV_CFL_PRED :
                    UV_DC_PRED;

            candidateArray[canTotalCnt].intra_chroma_mode = disable_ang_uv && av1_is_directional_mode(candidateArray[canTotalCnt].intra_chroma_mode) ?
                UV_DC_PRED : candidateArray[canTotalCnt].intra_chroma_mode;
#endif
#if CHROMA_DC_ONLY
            candidateArray[canTotalCnt].intra_chroma_mode = UV_DC_PRED;
#if CHROMA_SEARCH_FIX
            candidateArray[canTotalCnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)candidateArray[canTotalCnt].intra_chroma_mode);
            candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = 0;
#endif
#endif
            candidateArray[canTotalCnt].cfl_alpha_signs = 0;
            candidateArray[canTotalCnt].cfl_alpha_idx = 0;
#if !CHROMA_SEARCH_FIX
            candidateArray[canTotalCnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)candidateArray[canTotalCnt].intra_chroma_mode);
            candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = 0;
#endif
#if ATB_TX_TYPE_SUPPORT_PER_TU
            candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;

            if (candidateArray[canTotalCnt].intra_chroma_mode == UV_CFL_PRED)
                candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
            else
                candidateArray[canTotalCnt].transform_type_uv =
#else
            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;

            if (candidateArray[canTotalCnt].intra_chroma_mode == UV_CFL_PRED)
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
            else
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] =
#endif
                av1_get_tx_type(
                    context_ptr->blk_geom->bsize,
                    0,
                    (PredictionMode)candidateArray[canTotalCnt].intra_luma_mode,
                    (UvPredictionMode)candidateArray[canTotalCnt].intra_chroma_mode,
                    PLANE_TYPE_UV,
                    0,
                    0,
                    0,
#if ATB_SUPPORT
                    context_ptr->blk_geom->txsize_uv[0][0],
#else
                    context_ptr->blk_geom->txsize_uv[0],
#endif
                    picture_control_set_ptr->parent_pcs_ptr->reduced_tx_set_used);
            candidateArray[canTotalCnt].ref_frame_type = INTRA_FRAME;
            candidateArray[canTotalCnt].pred_mode = (PredictionMode)openLoopIntraCandidate;
            candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
#if CHECK_CAND
            INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
            ++canTotalCnt;
#endif
        }
    }

    // update the total number of candidates injected
    (*candidateTotalCnt) = canTotalCnt;

    return;
}
#if FI_MD

static const PredictionMode fimode_to_intramode[FILTER_INTRA_MODES] = {
  DC_PRED, V_PRED, H_PRED, D157_PRED, PAETH_PRED
};
// END of Function Declarations
void  inject_filter_intra_candidates(
    PictureControlSet            *picture_control_set_ptr,
    ModeDecisionContext          *context_ptr,
    const SequenceControlSet     *sequence_control_set_ptr,
    LargestCodingUnit            *sb_ptr,
#if M8_SKIP_BLK
    uint32_t                       *candidateTotalCnt){
#else
    uint32_t                       *candidateTotalCnt,
    const uint32_t                  leaf_index){
    (void)leaf_index;
#endif
    (void)sequence_control_set_ptr;
    (void)sb_ptr;
    uint8_t                     is16bit = (sequence_control_set_ptr->static_config.encoder_bit_depth > EB_8BIT);
    FilterIntraMode             intra_mode_start = FILTER_DC_PRED;
    FilterIntraMode             intra_mode_end   = FILTER_INTRA_MODES;
    FilterIntraMode             filter_intra_mode;
    uint32_t                    canTotalCnt = *candidateTotalCnt;
    uint8_t                     angleDeltaCounter = 0;
    EbBool                      use_angle_delta = (context_ptr->blk_geom->bsize >= BLOCK_8X8);
    uint8_t                     angleDeltaCandidateCount = use_angle_delta ? 7 : 1;
    ModeDecisionCandidate    *candidateArray = context_ptr->fast_candidate_array;
#if DISABLE_CFL
                EbBool disable_cfl_flag = 1;
#else
#if CFL_FIX
    EbBool                      disable_cfl_flag = (MAX(context_ptr->blk_geom->bheight, context_ptr->blk_geom->bwidth) > 32) ? EB_TRUE : EB_FALSE;
#else
    EbBool                      disable_cfl_flag = (context_ptr->blk_geom->sq_size > 32 ||
                                                    context_ptr->blk_geom->bwidth == 4  ||
                                                    context_ptr->blk_geom->bheight == 4)    ? EB_TRUE : EB_FALSE;
#endif
#endif
    /*if (disable_cfl_flag)
        printf("\nSTOPPPPPPPPP");*/

    for (filter_intra_mode = intra_mode_start; filter_intra_mode < intra_mode_end ; ++filter_intra_mode) {


            candidateArray[canTotalCnt].type = INTRA_MODE;
            candidateArray[canTotalCnt].intra_luma_mode = DC_PRED;
            candidateArray[canTotalCnt].distortion_ready = 0;
            candidateArray[canTotalCnt].use_intrabc = 0;
            candidateArray[canTotalCnt].filter_intra_mode = filter_intra_mode;
            candidateArray[canTotalCnt].is_directional_mode_flag = 0;
#if !SEARCH_UV_CLEAN_UP
            candidateArray[canTotalCnt].use_angle_delta = candidateArray[canTotalCnt].is_directional_mode_flag;
#endif
            candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y] = 0;
#if SEARCH_UV_MODE
            // Search the best independent intra chroma mode
            if (context_ptr->chroma_level == CHROMA_MODE_0) {
                candidateArray[canTotalCnt].intra_chroma_mode  = disable_cfl_flag ? context_ptr->best_uv_mode[fimode_to_intramode[filter_intra_mode]][MAX_ANGLE_DELTA + candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y]] : UV_CFL_PRED;
#if CHROMA_SEARCH_FIX
                candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = disable_cfl_flag ? context_ptr->best_uv_angle[fimode_to_intramode[filter_intra_mode]][MAX_ANGLE_DELTA + candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y]] : 0;
                candidateArray[canTotalCnt].is_directional_chroma_mode_flag = disable_cfl_flag ? (uint8_t)av1_is_directional_mode((PredictionMode)(context_ptr->best_uv_mode[fimode_to_intramode[filter_intra_mode]][MAX_ANGLE_DELTA + candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_Y]])) : 0;
#endif
            }
            else {
                // Hsan/Omar: why the restriction below ? (i.e. disable_ang_uv)
                const int32_t disable_ang_uv = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) && context_ptr->blk_geom->has_uv ? 1 : 0;
                candidateArray[canTotalCnt].intra_chroma_mode = disable_cfl_flag ? intra_luma_to_chroma[fimode_to_intramode[filter_intra_mode]] :
                    (context_ptr->chroma_level == CHROMA_MODE_1) ?
                        UV_CFL_PRED :
                        UV_DC_PRED;

                candidateArray[canTotalCnt].intra_chroma_mode =  disable_ang_uv && av1_is_directional_mode(candidateArray[canTotalCnt].intra_chroma_mode) ?
                    UV_DC_PRED : candidateArray[canTotalCnt].intra_chroma_mode;

#if CHROMA_SEARCH_FIX
                candidateArray[canTotalCnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)candidateArray[canTotalCnt].intra_chroma_mode);
                candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = 0;
#endif

            }
#else
            const int32_t disable_ang_uv = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) && context_ptr->blk_geom->has_uv ? 1 : 0;
            candidateArray[canTotalCnt].intra_chroma_mode = disable_cfl_flag ?
                intra_luma_to_chroma[openLoopIntraCandidate] :
                (context_ptr->chroma_level <= CHROMA_MODE_1) ?
                    UV_CFL_PRED :
                    UV_DC_PRED;

            candidateArray[canTotalCnt].intra_chroma_mode = disable_ang_uv && av1_is_directional_mode(candidateArray[canTotalCnt].intra_chroma_mode) ?
                UV_DC_PRED : candidateArray[canTotalCnt].intra_chroma_mode;
#endif
#if CHROMA_DC_ONLY
            candidateArray[canTotalCnt].intra_chroma_mode = UV_DC_PRED;
#if CHROMA_SEARCH_FIX
            candidateArray[canTotalCnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)candidateArray[canTotalCnt].intra_chroma_mode);
            candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = 0;
#endif
#endif
            candidateArray[canTotalCnt].cfl_alpha_signs = 0;
            candidateArray[canTotalCnt].cfl_alpha_idx = 0;
#if !CHROMA_SEARCH_FIX
            candidateArray[canTotalCnt].is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)candidateArray[canTotalCnt].intra_chroma_mode);
            candidateArray[canTotalCnt].angle_delta[PLANE_TYPE_UV] = 0;
#endif
#if ATB_TX_TYPE_SUPPORT_PER_TU
            candidateArray[canTotalCnt].transform_type[0] = DCT_DCT;

            if (candidateArray[canTotalCnt].intra_chroma_mode == UV_CFL_PRED)
                candidateArray[canTotalCnt].transform_type_uv = DCT_DCT;
            else
                candidateArray[canTotalCnt].transform_type_uv =
#else
            candidateArray[canTotalCnt].transform_type[PLANE_TYPE_Y] = DCT_DCT;

            if (candidateArray[canTotalCnt].intra_chroma_mode == UV_CFL_PRED)
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] = DCT_DCT;
            else
                candidateArray[canTotalCnt].transform_type[PLANE_TYPE_UV] =
#endif
                av1_get_tx_type(
                    context_ptr->blk_geom->bsize,
                    0,
                    (PredictionMode)candidateArray[canTotalCnt].intra_luma_mode,
                    (UvPredictionMode)candidateArray[canTotalCnt].intra_chroma_mode,
                    PLANE_TYPE_UV,
                    0,
                    0,
                    0,
#if ATB_SUPPORT
                    context_ptr->blk_geom->txsize_uv[0][0],
#else
                    context_ptr->blk_geom->txsize_uv[0],
#endif
                    picture_control_set_ptr->parent_pcs_ptr->reduced_tx_set_used);
            candidateArray[canTotalCnt].ref_frame_type = INTRA_FRAME;
            candidateArray[canTotalCnt].pred_mode = DC_PRED;
            candidateArray[canTotalCnt].motion_mode = SIMPLE_TRANSLATION;
#if CHECK_CAND
            INCRMENT_CAND_TOTAL_COUNT(canTotalCnt);
#else
            ++canTotalCnt;
#endif

    }

    // update the total number of candidates injected
    (*candidateTotalCnt) = canTotalCnt;

    return;
}

#endif
#if !OPT_LOSSLESS_0
void ProductInitMdCandInjection(
    ModeDecisionContext          *context_ptr,
    uint32_t                         *candidateTotalCnt)

{
    *candidateTotalCnt = 0;
    context_ptr->generate_mvp = EB_FALSE;

    return;
}
#endif
#if MD_CLASS
EbErrorType generate_md_stage_0_cand(
#else
/***************************************
* ProductGenerateMdCandidatesCu
*   Creates list of initial modes to
*   perform fast cost search on.
***************************************/
EbErrorType ProductGenerateMdCandidatesCu(
#endif
    LargestCodingUnit                 *sb_ptr,
    ModeDecisionContext             *context_ptr,
    SsMeContext                    *ss_mecontext,
#if !M8_SKIP_BLK
    const uint32_t                      leaf_index,
#endif
    const uint32_t                      lcuAddr,
    uint32_t                           *candidateTotalCountPtr,
    EbPtr                              interPredContextPtr,
    PictureControlSet              *picture_control_set_ptr)
{
    (void)lcuAddr;
    (void)interPredContextPtr;
    const SequenceControlSet *sequence_control_set_ptr = (SequenceControlSet*)picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr;
    const EB_SLICE slice_type = picture_control_set_ptr->slice_type;
#if OPT_LOSSLESS_0
    uint32_t canTotalCnt = 0;
#else
    uint32_t       canTotalCnt;
#endif
    // Reset duplicates variables
    context_ptr->injected_mv_count_l0 = 0;
    context_ptr->injected_mv_count_l1 = 0;
    context_ptr->injected_mv_count_bipred = 0;
#if !OPT_LOSSLESS_0
    ProductInitMdCandInjection(
        context_ptr,
        &canTotalCnt);
#endif
    uint8_t sq_index = LOG2F(context_ptr->blk_geom->sq_size) - 2;
    uint8_t inject_intra_candidate = 1;
    uint8_t inject_inter_candidate = 1;

    if (slice_type != I_SLICE) {
#if ADP_BQ
        // to add the support for extra partitioning method here
#endif
#if ADP_BQ
        if (picture_control_set_ptr->parent_pcs_ptr->pic_depth_mode == PIC_SB_SWITCH_NSQ_DEPTH_MODE || (picture_control_set_ptr->parent_pcs_ptr->nsq_search_level >= NSQ_SEARCH_LEVEL1 && picture_control_set_ptr->parent_pcs_ptr->nsq_search_level < NSQ_SEARCH_FULL)) {
#else
        if (picture_control_set_ptr->parent_pcs_ptr->nsq_search_level >= NSQ_SEARCH_LEVEL1 &&
            picture_control_set_ptr->parent_pcs_ptr->nsq_search_level < NSQ_SEARCH_FULL) {
#endif
            inject_intra_candidate = context_ptr->blk_geom->shape == PART_N ? 1 :
                context_ptr->parent_sq_has_coeff[sq_index] != 0 ? inject_intra_candidate : 0;
        }
     }
#if AVOID_INTER_4X4_CHROMA
    inject_inter_candidate = (context_ptr->blk_geom->has_uv && (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4)) ? 0 : inject_inter_candidate;
    inject_intra_candidate = (context_ptr->blk_geom->has_uv && (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4)) ? 1 : inject_intra_candidate;
#endif
    //----------------------
    // Intra
    if (context_ptr->blk_geom->sq_size < 128) {
#if M9_INTRA
        if (picture_control_set_ptr->parent_pcs_ptr->intra_pred_mode >= 5 && context_ptr->blk_geom->sq_size > 4 && context_ptr->blk_geom->shape == PART_N)
#endif
            inject_intra_candidates_ois(
                picture_control_set_ptr,
                context_ptr,
                sb_ptr,
                &canTotalCnt);
        else
            if (inject_intra_candidate)
            inject_intra_candidates(
                picture_control_set_ptr,
                context_ptr,
                sequence_control_set_ptr,
                sb_ptr,
#if M8_SKIP_BLK
                &canTotalCnt);
#else
                &canTotalCnt,
                leaf_index);
#endif
#if FI_MD
       if (picture_control_set_ptr->pic_filter_intra_mode > 0 && av1_filter_intra_allowed_bsize(sequence_control_set_ptr->seq_header.enable_filter_intra, context_ptr->blk_geom->bsize))

            inject_filter_intra_candidates(
                picture_control_set_ptr,
                context_ptr,
                sequence_control_set_ptr,
                sb_ptr,
#if M8_SKIP_BLK
                &canTotalCnt);
#else
                &canTotalCnt,
                leaf_index);
#endif
#endif
    }

    if (picture_control_set_ptr->parent_pcs_ptr->allow_intrabc)
        inject_intra_bc_candidates(
            picture_control_set_ptr,
            context_ptr,
            sequence_control_set_ptr,
            //sb_ptr,
            context_ptr->cu_ptr,
            &canTotalCnt
        );

    // Track the total number of fast intra candidates
    context_ptr->fast_candidate_intra_count = canTotalCnt;

    if (slice_type != I_SLICE) {
        if (inject_inter_candidate)
            inject_inter_candidates(
                picture_control_set_ptr,
                context_ptr,
                ss_mecontext,
                sequence_control_set_ptr,
                sb_ptr,
#if M8_SKIP_BLK
                &canTotalCnt);
#else
                &canTotalCnt,
                leaf_index);
#endif
    }
    *candidateTotalCountPtr = canTotalCnt;


#if MD_CLASS

    CAND_CLASS  cand_class_it;
    memset(context_ptr->fast_cand_count, 0, CAND_CLASS_TOTAL * sizeof(uint32_t));

    uint32_t cand_i;
    for (cand_i = 0; cand_i < canTotalCnt; cand_i++)
    {
        ModeDecisionCandidate * cand_ptr = &context_ptr->fast_candidate_array[cand_i];

            if (cand_ptr->type == INTRA_MODE) {
#if FI_CLASS
                if (cand_ptr->filter_intra_mode == FILTER_INTRA_MODES) {
                    cand_ptr->cand_class = CAND_CLASS_0;
                    context_ptr->fast_cand_count[CAND_CLASS_0]++;
                }
                else {
                    cand_ptr->cand_class = CAND_CLASS_5;
                    context_ptr->fast_cand_count[CAND_CLASS_5]++;
                }
#else
                cand_ptr->cand_class = CAND_CLASS_0;
                context_ptr->fast_cand_count[CAND_CLASS_0]++;
#endif
            }
#if COMP_FULL
            else if ((cand_ptr->type == INTER_MODE && cand_ptr->is_compound == 0) ||
                (cand_ptr->type == INTER_MODE && cand_ptr->is_compound == 1 && cand_ptr->interinter_comp.type == COMPOUND_AVERAGE)) {

#else
            else
#endif
#if II_CLASS
                if (cand_ptr->is_interintra_used && cand_ptr->is_compound == 0) {
                    cand_ptr->cand_class = CAND_CLASS_4;
                    context_ptr->fast_cand_count[CAND_CLASS_4]++;

                }else
#endif

#if AUTO_C1C2
                    if (context_ptr->combine_class12){
                        cand_ptr->cand_class = CAND_CLASS_1;
                        context_ptr->fast_cand_count[CAND_CLASS_1]++;
                    }
                    else {
                        if (cand_ptr->is_new_mv) {
                            cand_ptr->cand_class = CAND_CLASS_1;
                            context_ptr->fast_cand_count[CAND_CLASS_1]++;
                        }
                        else {
                            cand_ptr->cand_class = CAND_CLASS_2;
                            context_ptr->fast_cand_count[CAND_CLASS_2]++;
                        }
                    }
#else
#if COMBINE_C1_C2
                cand_ptr->cand_class = CAND_CLASS_1;
                context_ptr->fast_cand_count[CAND_CLASS_1]++;
#else
                if (cand_ptr->is_new_mv) {
                    cand_ptr->cand_class = CAND_CLASS_1;
                    context_ptr->fast_cand_count[CAND_CLASS_1]++;
                }
                else {
                    cand_ptr->cand_class = CAND_CLASS_2;
                    context_ptr->fast_cand_count[CAND_CLASS_2]++;
                }
#endif
#endif
#if COMP_FULL
            }
            else {

#if AUTO_C1C2
        if (context_ptr->combine_class12) {
            cand_ptr->cand_class = CAND_CLASS_2;
            context_ptr->fast_cand_count[CAND_CLASS_2]++;
        }
        else {
            cand_ptr->cand_class = CAND_CLASS_3;
            context_ptr->fast_cand_count[CAND_CLASS_3]++;
        }
#else
#if COMBINE_C1_C2
                cand_ptr->cand_class = CAND_CLASS_2;
                context_ptr->fast_cand_count[CAND_CLASS_2]++;
#else
                cand_ptr->cand_class = CAND_CLASS_3;
                context_ptr->fast_cand_count[CAND_CLASS_3]++;
#endif
#endif
            }
#endif
    }

    uint32_t fast_accum = 0;
    for (cand_class_it = CAND_CLASS_0; cand_class_it < CAND_CLASS_TOTAL; cand_class_it++) {
        fast_accum += context_ptr->fast_cand_count[cand_class_it];
    }
    assert(fast_accum == canTotalCnt);
#endif


    return EB_ErrorNone;
}

/***************************************
* Full Mode Decision
***************************************/
#if DECOUPLED_FAST_LOOP || MD_CLASS
    uint32_t product_full_mode_decision(
    struct ModeDecisionContext   *context_ptr,
    CodingUnit                   *cu_ptr,
    uint8_t                           bwidth,
    uint8_t                           bheight,
    ModeDecisionCandidateBuffer **buffer_ptr_array,
    uint32_t                          candidate_total_count,
    uint32_t                          *best_candidate_index_array,
#if PRUNE_REF_FRAME_FRO_REC_PARTITION
    uint8_t                            prune_ref_frame_for_rec_partitions,
#endif
    uint32_t                           *best_intra_mode)
#else
uint8_t product_full_mode_decision(
    struct ModeDecisionContext   *context_ptr,
    CodingUnit                   *cu_ptr,
    uint8_t                           bwidth,
    uint8_t                           bheight,
    ModeDecisionCandidateBuffer **buffer_ptr_array,
    uint32_t                          candidate_total_count,
    uint8_t                          *best_candidate_index_array,
    uint32_t                           *best_intra_mode)
#endif
{
    UNUSED(bwidth);
    UNUSED(bheight);
#if DECOUPLED_FAST_LOOP || MD_CLASS
    uint32_t                   candidateIndex;
    uint64_t                  lowestCost = 0xFFFFFFFFFFFFFFFFull;
    uint64_t                  lowestIntraCost = 0xFFFFFFFFFFFFFFFFull;
    uint32_t                   lowestCostIndex = 0;
#else
    uint8_t                   candidateIndex;
    uint64_t                  lowestCost = 0xFFFFFFFFFFFFFFFFull;
    uint64_t                  lowestIntraCost = 0xFFFFFFFFFFFFFFFFull;
    uint8_t                   lowestCostIndex = 0;
#endif
    PredictionUnit       *pu_ptr;
    uint32_t                   i;
    ModeDecisionCandidate       *candidate_ptr;

    lowestCostIndex = best_candidate_index_array[0];

#if PRUNE_REF_FRAME_FRO_REC_PARTITION
    if (prune_ref_frame_for_rec_partitions) {
        if (context_ptr->blk_geom->shape == PART_N) {
            for (i = 0; i < candidate_total_count; ++i) {
                candidateIndex = best_candidate_index_array[i];
                candidate_ptr = buffer_ptr_array[candidateIndex]->candidate_ptr;
                EbBool is_inter = (candidate_ptr->pred_mode >= NEARESTMV) ? EB_TRUE : EB_FALSE;
                EbBool is_simple_translation = (candidate_ptr->motion_mode != WARPED_CAUSAL) ? EB_TRUE : EB_FALSE;
                if (is_inter && is_simple_translation) {
                    uint8_t ref_frame_type = candidate_ptr->ref_frame_type;
                    assert(ref_frame_type < MAX_REF_TYPE_CAND);
                    context_ptr->ref_best_cost_sq_table[ref_frame_type] = *(buffer_ptr_array[candidateIndex]->full_cost_ptr);
                }

            }
            //Sort ref_frame by sq cost.
            uint32_t i, j, index;
            for (i = 0; i < MAX_REF_TYPE_CAND; ++i) {
                context_ptr->ref_best_ref_sq_table[i] = i;
            }
            for (i = 0; i < MAX_REF_TYPE_CAND - 1; ++i) {
                for (j = i + 1; j < MAX_REF_TYPE_CAND; ++j) {
                    if (context_ptr->ref_best_cost_sq_table[context_ptr->ref_best_ref_sq_table[j]] < context_ptr->ref_best_cost_sq_table[context_ptr->ref_best_ref_sq_table[i]]) {
                        index = context_ptr->ref_best_ref_sq_table[i];
                        context_ptr->ref_best_ref_sq_table[i] = context_ptr->ref_best_ref_sq_table[j];
                        context_ptr->ref_best_ref_sq_table[j] = index;
                    }
                }
            }
        }
    }
#endif
    // Find the candidate with the lowest cost
    for (i = 0; i < candidate_total_count; ++i) {
        candidateIndex = best_candidate_index_array[i];

        // Compute fullCostBis
        if ((*(buffer_ptr_array[candidateIndex]->full_cost_ptr) < lowestIntraCost) && buffer_ptr_array[candidateIndex]->candidate_ptr->type == INTRA_MODE) {
            *best_intra_mode = buffer_ptr_array[candidateIndex]->candidate_ptr->pred_mode;
            lowestIntraCost = *(buffer_ptr_array[candidateIndex]->full_cost_ptr);
        }

        if (*(buffer_ptr_array[candidateIndex]->full_cost_ptr) < lowestCost) {
            lowestCostIndex = candidateIndex;
            lowestCost = *(buffer_ptr_array[candidateIndex]->full_cost_ptr);
        }
    }

    candidate_ptr = buffer_ptr_array[lowestCostIndex]->candidate_ptr;

    context_ptr->md_local_cu_unit[cu_ptr->mds_idx].cost = *(buffer_ptr_array[lowestCostIndex]->full_cost_ptr);
    context_ptr->md_local_cu_unit[cu_ptr->mds_idx].cost = (context_ptr->md_local_cu_unit[cu_ptr->mds_idx].cost - buffer_ptr_array[lowestCostIndex]->candidate_ptr->chroma_distortion) + buffer_ptr_array[lowestCostIndex]->candidate_ptr->chroma_distortion_inter_depth;

    if (candidate_ptr->type == INTRA_MODE)
        context_ptr->md_local_cu_unit[cu_ptr->mds_idx].cost_luma = buffer_ptr_array[lowestCostIndex]->full_cost_luma;

    context_ptr->md_ep_pipe_sb[cu_ptr->mds_idx].merge_cost = *buffer_ptr_array[lowestCostIndex]->full_cost_merge_ptr;
    context_ptr->md_ep_pipe_sb[cu_ptr->mds_idx].skip_cost = *buffer_ptr_array[lowestCostIndex]->full_cost_skip_ptr;

    if (candidate_ptr->type == INTER_MODE && candidate_ptr->merge_flag == EB_TRUE)
        context_ptr->md_ep_pipe_sb[cu_ptr->mds_idx].chroma_distortion = buffer_ptr_array[lowestCostIndex]->candidate_ptr->chroma_distortion;
    context_ptr->md_local_cu_unit[cu_ptr->mds_idx].full_distortion = buffer_ptr_array[lowestCostIndex]->candidate_ptr->full_distortion;
    context_ptr->md_local_cu_unit[cu_ptr->mds_idx].chroma_distortion = (uint32_t)buffer_ptr_array[lowestCostIndex]->candidate_ptr->chroma_distortion;
    context_ptr->md_local_cu_unit[cu_ptr->mds_idx].chroma_distortion_inter_depth = (uint32_t)buffer_ptr_array[lowestCostIndex]->candidate_ptr->chroma_distortion_inter_depth;

    cu_ptr->prediction_mode_flag = candidate_ptr->type;
#if ATB_SUPPORT
    cu_ptr->tx_depth = candidate_ptr->tx_depth;
#endif
    cu_ptr->skip_flag = candidate_ptr->skip_flag; // note, the skip flag is re-checked in the ENCDEC process
    cu_ptr->block_has_coeff = ((candidate_ptr->block_has_coeff) > 0) ? EB_TRUE : EB_FALSE;
#if ATB_DC_CONTEXT_SUPPORT_1
    cu_ptr->quantized_dc[1][0] = buffer_ptr_array[lowestCostIndex]->candidate_ptr->quantized_dc[1][0];
    cu_ptr->quantized_dc[2][0] = buffer_ptr_array[lowestCostIndex]->candidate_ptr->quantized_dc[2][0];
#else
    cu_ptr->quantized_dc[0] = buffer_ptr_array[lowestCostIndex]->candidate_ptr->quantized_dc[0];
    cu_ptr->quantized_dc[1] = buffer_ptr_array[lowestCostIndex]->candidate_ptr->quantized_dc[1];
    cu_ptr->quantized_dc[2] = buffer_ptr_array[lowestCostIndex]->candidate_ptr->quantized_dc[2];
#endif
    context_ptr->md_local_cu_unit[cu_ptr->mds_idx].count_non_zero_coeffs = candidate_ptr->count_non_zero_coeffs;

    cu_ptr->av1xd->use_intrabc = candidate_ptr->use_intrabc;
#if COMP_MODE
    if (cu_ptr->prediction_mode_flag == INTER_MODE && candidate_ptr->is_compound)
    {
        cu_ptr->interinter_comp.type = candidate_ptr->interinter_comp.type;
        cu_ptr->interinter_comp.mask_type = candidate_ptr->interinter_comp.mask_type;
        cu_ptr->interinter_comp.wedge_index = candidate_ptr->interinter_comp.wedge_index;
        cu_ptr->interinter_comp.wedge_sign = candidate_ptr->interinter_comp.wedge_sign;
        //mmecpy? cu_ptr->interinter_comp.seg_mask = candidate_ptr->interinter_comp.seg_mask;
        cu_ptr->compound_idx = candidate_ptr->compound_idx;
        cu_ptr->comp_group_idx = candidate_ptr->comp_group_idx;
        if (cu_ptr->interinter_comp.type == COMPOUND_AVERAGE)
        {
            if (cu_ptr->comp_group_idx != 0 || cu_ptr->compound_idx != 1)
                printf("NOPPLLLL");
        }
        else {
           //printf("NOPPLLLL");
        }

        //this TEMP
        //if (/*context_ptr->blk_geom->bsize==BLOCK_16X16 &&*/ wedge_params_lookup[context_ptr->blk_geom->bsize].bits > 0 && candidate_ptr->merge_flag==0 && cu_ptr->interinter_comp.type == COMPOUND_AVERAGE) {
        //    //cu_ptr->interinter_comp.type = COMPOUND_DIFFWTD;
        //    //cu_ptr->comp_group_idx = 1;
        //    //cu_ptr->compound_idx = 1;
        //    //cu_ptr->interinter_comp.mask_type = 0;

        //    cu_ptr->interinter_comp.type = COMPOUND_WEDGE;
        //    cu_ptr->comp_group_idx = 1;
        //    cu_ptr->compound_idx = 1;
        //    cu_ptr->interinter_comp.wedge_index =  context_ptr->blk_geom->sq_size <= 16 ? 3 : 10;
        //    cu_ptr->interinter_comp.wedge_sign = 0;

        //}
    }
#endif
#if II_COMP

    cu_ptr->is_interintra_used          = candidate_ptr->is_interintra_used;
    cu_ptr->interintra_mode             = candidate_ptr->interintra_mode;
    cu_ptr->use_wedge_interintra        = candidate_ptr->use_wedge_interintra;
    cu_ptr->interintra_wedge_index      = candidate_ptr->interintra_wedge_index;//inter_intra wedge index
    cu_ptr->ii_wedge_sign               = candidate_ptr->ii_wedge_sign;//inter_intra wedge sign=-1

#endif
    // Set the PU level variables
    cu_ptr->interp_filters = candidate_ptr->interp_filters;
    {
        pu_ptr = cu_ptr->prediction_unit_array;
        // Intra Prediction
        pu_ptr->intra_luma_mode = 0x1F;
        if (cu_ptr->prediction_mode_flag == INTRA_MODE)
        {
#if FI_ED
            cu_ptr->filter_intra_mode= candidate_ptr->filter_intra_mode;
#endif
            pu_ptr->intra_luma_mode = candidate_ptr->intra_luma_mode;

            pu_ptr->is_directional_mode_flag = candidate_ptr->is_directional_mode_flag;
#if !SEARCH_UV_CLEAN_UP
            pu_ptr->use_angle_delta = candidate_ptr->use_angle_delta;
#endif
            pu_ptr->angle_delta[PLANE_TYPE_Y] = candidate_ptr->angle_delta[PLANE_TYPE_Y];

            pu_ptr->cfl_alpha_idx = candidate_ptr->cfl_alpha_idx;
            pu_ptr->cfl_alpha_signs = candidate_ptr->cfl_alpha_signs;

            pu_ptr->intra_chroma_mode = candidate_ptr->intra_chroma_mode;
            pu_ptr->is_directional_chroma_mode_flag = candidate_ptr->is_directional_chroma_mode_flag;
            pu_ptr->angle_delta[PLANE_TYPE_UV] = candidate_ptr->angle_delta[PLANE_TYPE_UV];
        }

        // Inter Prediction
        pu_ptr->inter_pred_direction_index = candidate_ptr->prediction_direction[0];
        pu_ptr->merge_flag = candidate_ptr->merge_flag;
        if (cu_ptr->prediction_mode_flag != INTER_MODE && cu_ptr->av1xd->use_intrabc == 0)
        {
            pu_ptr->inter_pred_direction_index = 0x03;
            pu_ptr->merge_flag = EB_FALSE;
        }
        pu_ptr->mv[REF_LIST_0].x = 0;
        pu_ptr->mv[REF_LIST_0].y = 0;

        pu_ptr->mv[REF_LIST_1].x = 0;
        pu_ptr->mv[REF_LIST_1].y = 0;

        cu_ptr->pred_mode = candidate_ptr->pred_mode;
        cu_ptr->drl_index = candidate_ptr->drl_index;

        pu_ptr->inter_mode = candidate_ptr->inter_mode;
        pu_ptr->is_compound = candidate_ptr->is_compound;
        pu_ptr->pred_mv_weight = candidate_ptr->pred_mv_weight;
        pu_ptr->ref_frame_type = candidate_ptr->ref_frame_type;
#if MRP_MD
        pu_ptr->ref_frame_index_l0 = candidate_ptr->ref_frame_index_l0;
        pu_ptr->ref_frame_index_l1 = candidate_ptr->ref_frame_index_l1;
#endif
        pu_ptr->ref_mv_index = candidate_ptr->ref_mv_index;
        pu_ptr->is_new_mv = candidate_ptr->is_new_mv;
        pu_ptr->is_zero_mv = candidate_ptr->is_zero_mv;

        if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_0)
        {
            //EB_MEMCPY(&pu_ptr->mv[REF_LIST_0].x,&candidate_ptr->mvs_l0,4);
            pu_ptr->mv[REF_LIST_0].x = candidate_ptr->motion_vector_xl0;
            pu_ptr->mv[REF_LIST_0].y = candidate_ptr->motion_vector_yl0;
        }

        if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_1)
        {
            //EB_MEMCPY(&pu_ptr->mv[REF_LIST_1].x,&candidate_ptr->mvs_l1,4);
            pu_ptr->mv[REF_LIST_1].x = candidate_ptr->motion_vector_xl1;
            pu_ptr->mv[REF_LIST_1].y = candidate_ptr->motion_vector_yl1;
        }

        if (pu_ptr->inter_pred_direction_index == BI_PRED)
        {
            //EB_MEMCPY(&pu_ptr->mv[REF_LIST_0].x,&candidate_ptr->mvs,8);
            pu_ptr->mv[REF_LIST_0].x = candidate_ptr->motion_vector_xl0;
            pu_ptr->mv[REF_LIST_0].y = candidate_ptr->motion_vector_yl0;
            pu_ptr->mv[REF_LIST_1].x = candidate_ptr->motion_vector_xl1;
            pu_ptr->mv[REF_LIST_1].y = candidate_ptr->motion_vector_yl1;
        }
#if NEW_NEAR_FIX
        if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_0) {
            cu_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_0];
            cu_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_0];
        }
        else if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_1) {
            cu_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_1];
            cu_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_1];
        }
        else if (pu_ptr->inter_pred_direction_index == BI_PRED) {
            cu_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_0];
            cu_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_0];
            cu_ptr->predmv[1].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_1];
            cu_ptr->predmv[1].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_1];
        }
#endif

        // The MV prediction indicies are recalcated by the EncDec.
        pu_ptr->mvd[REF_LIST_0].pred_idx = 0;
        pu_ptr->mvd[REF_LIST_1].pred_idx = 0;

        pu_ptr->overlappable_neighbors[0] = context_ptr->cu_ptr->prediction_unit_array[0].overlappable_neighbors[0];
        pu_ptr->overlappable_neighbors[1] = context_ptr->cu_ptr->prediction_unit_array[0].overlappable_neighbors[1];
        pu_ptr->motion_mode = candidate_ptr->motion_mode;
        pu_ptr->num_proj_ref = candidate_ptr->num_proj_ref;
        if (pu_ptr->motion_mode == WARPED_CAUSAL)
            EB_MEMCPY(&pu_ptr->wm_params, &candidate_ptr->wm_params, sizeof(EbWarpedMotionParams));
    }

    TransformUnit *txb_ptr;
    uint32_t txb_itr;
    uint32_t tu_index;
    uint32_t tuTotalCount;
    uint32_t cu_size_log2 = context_ptr->cu_size_log2;
#if ATB_SUPPORT
    tuTotalCount = context_ptr->blk_geom->txb_count[cu_ptr->tx_depth];
    tu_index = 0;
    txb_itr = 0;
#else
    {
        tuTotalCount = context_ptr->blk_geom->txb_count;
        tu_index = 0;
        txb_itr = 0;
    }
#endif
#if NO_ENCDEC
    int32_t txb_1d_offset = 0, txb_1d_offset_uv = 0;

    cu_ptr->block_has_coeff = 0;
#endif

    //cu_ptr->forceSmallTu = candidate_ptr->forceSmallTu;

    // Set TU
    do {
        txb_ptr = &cu_ptr->transform_unit_array[tu_index];

        txb_ptr->split_flag = EB_FALSE;
        txb_ptr->y_has_coeff = (EbBool)(((candidate_ptr->y_has_coeff)  & (1 << tu_index)) > 0);
        txb_ptr->u_has_coeff = (EbBool)(((candidate_ptr->u_has_coeff) & (1 << (tu_index))) > 0);
        txb_ptr->v_has_coeff = (EbBool)(((candidate_ptr->v_has_coeff) & (1 << (tu_index))) > 0);
#if ATB_TX_TYPE_SUPPORT_PER_TU
        txb_ptr->transform_type[PLANE_TYPE_Y] = candidate_ptr->transform_type[tu_index];
        txb_ptr->transform_type[PLANE_TYPE_UV] = candidate_ptr->transform_type_uv;
#else
        txb_ptr->transform_type[PLANE_TYPE_Y] = candidate_ptr->transform_type[PLANE_TYPE_Y];
        txb_ptr->transform_type[PLANE_TYPE_UV] = candidate_ptr->transform_type[PLANE_TYPE_UV];
#endif

#if ATB_DC_CONTEXT_SUPPORT_1
        cu_ptr->quantized_dc[0][tu_index] = candidate_ptr->quantized_dc[0][tu_index];
#endif

#if NO_ENCDEC

        if (context_ptr->blk_geom->has_uv) {
            cu_ptr->block_has_coeff |= txb_ptr->y_has_coeff;
            cu_ptr->block_has_coeff |= txb_ptr->u_has_coeff;
            cu_ptr->block_has_coeff |= txb_ptr->v_has_coeff;
        }
        else
            cu_ptr->block_has_coeff |= txb_ptr->y_has_coeff;
        cu_ptr->cand_buff_index = lowestCostIndex;

        cu_ptr->skip_flag = 0;   //SKIP is turned OFF for this case!!
        txb_ptr->nz_coef_count[0] = candidate_ptr->eob[0][tu_index];
        txb_ptr->nz_coef_count[1] = candidate_ptr->eob[1][tu_index];
        txb_ptr->nz_coef_count[2] = candidate_ptr->eob[2][tu_index];

        if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_0) {
            cu_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_0];
            cu_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_0];
        }
        else if (pu_ptr->inter_pred_direction_index == UNI_PRED_LIST_1) {
            cu_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_1];
            cu_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_1];
        }
        else if (pu_ptr->inter_pred_direction_index == BI_PRED) {
            cu_ptr->predmv[0].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_0];
            cu_ptr->predmv[0].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_0];
            cu_ptr->predmv[1].as_mv.col = candidate_ptr->motion_vector_pred_x[REF_LIST_1];
            cu_ptr->predmv[1].as_mv.row = candidate_ptr->motion_vector_pred_y[REF_LIST_1];
        }
#endif
#if NO_ENCDEC
        //copy coeff
        {
            uint32_t  bwidth = context_ptr->blk_geom->tx_width[txb_itr] < 64 ? context_ptr->blk_geom->tx_width[txb_itr] : 32;
            uint32_t  bheight = context_ptr->blk_geom->tx_height[txb_itr] < 64 ? context_ptr->blk_geom->tx_height[txb_itr] : 32;

            int32_t* src_ptr = &(((int32_t*)buffer_ptr_array[lowestCostIndex]->residual_quant_coeff_ptr->buffer_y)[txb_1d_offset]);
            int32_t* dst_ptr = &(((int32_t*)context_ptr->cu_ptr->coeff_tmp->buffer_y)[txb_1d_offset]);

            uint32_t j;

            for (j = 0; j < bheight; j++)
                memcpy(dst_ptr + j * bwidth, src_ptr + j * bwidth, bwidth * sizeof(int32_t));
            if (context_ptr->blk_geom->has_uv)
            {
                // Cb
                bwidth = context_ptr->blk_geom->tx_width_uv[txb_itr];
                bheight = context_ptr->blk_geom->tx_height_uv[txb_itr];

                src_ptr = &(((int32_t*)buffer_ptr_array[lowestCostIndex]->residual_quant_coeff_ptr->buffer_cb)[txb_1d_offset_uv]);
                dst_ptr = &(((int32_t*)context_ptr->cu_ptr->coeff_tmp->buffer_cb)[txb_1d_offset_uv]);

                for (j = 0; j < bheight; j++)
                    memcpy(dst_ptr + j * bwidth, src_ptr + j * bwidth, bwidth * sizeof(int32_t));
                src_ptr = &(((int32_t*)buffer_ptr_array[lowestCostIndex]->residual_quant_coeff_ptr->buffer_cr)[txb_1d_offset_uv]);
                dst_ptr = &(((int32_t*)context_ptr->cu_ptr->coeff_tmp->buffer_cr)[txb_1d_offset_uv]);

                for (j = 0; j < bheight; j++)
                    memcpy(dst_ptr + j * bwidth, src_ptr + j * bwidth, bwidth * sizeof(int32_t));
            }

            txb_1d_offset += context_ptr->blk_geom->tx_width[txb_itr] * context_ptr->blk_geom->tx_height[txb_itr];
            if (context_ptr->blk_geom->has_uv)
                txb_1d_offset_uv += context_ptr->blk_geom->tx_width_uv[txb_itr] * context_ptr->blk_geom->tx_height_uv[txb_itr];
        }

#endif

        ++tu_index;
        ++txb_itr;
    } while (txb_itr < tuTotalCount);
    UNUSED(cu_size_log2);
    return lowestCostIndex;
}
