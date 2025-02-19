/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#include <stdlib.h>

#include "EbDefinitions.h"
#include "EbEncodeContext.h"
#include "EbPictureManagerQueue.h"
#include "EbCabacContextModel.h"
#include "EbSvtAv1ErrorCodes.h"

static void encode_context_dctor(EbPtr p)
{
    EncodeContext* obj = (EncodeContext*)p;
    EB_DESTROY_MUTEX(obj->total_number_of_recon_frame_mutex);
    EB_DESTROY_MUTEX(obj->hl_rate_control_historgram_queue_mutex);
    EB_DESTROY_MUTEX(obj->rate_table_update_mutex);
    EB_DESTROY_MUTEX(obj->sc_buffer_mutex);
    EB_DESTROY_MUTEX(obj->shared_reference_mutex);

    EB_DELETE(obj->prediction_structure_group_ptr);
    EB_DELETE_PTR_ARRAY(obj->picture_decision_reorder_queue, PICTURE_DECISION_REORDER_QUEUE_MAX_DEPTH);
    EB_DELETE_PTR_ARRAY(obj->picture_manager_reorder_queue, PICTURE_MANAGER_REORDER_QUEUE_MAX_DEPTH);
    EB_FREE(obj->pre_assignment_buffer);
    EB_DELETE_PTR_ARRAY(obj->input_picture_queue, INPUT_QUEUE_MAX_DEPTH);
    EB_DELETE_PTR_ARRAY(obj->reference_picture_queue, REFERENCE_QUEUE_MAX_DEPTH);
    EB_DELETE_PTR_ARRAY(obj->picture_decision_pa_reference_queue, PICTURE_DECISION_PA_REFERENCE_QUEUE_MAX_DEPTH);
    EB_DELETE_PTR_ARRAY(obj->initial_rate_control_reorder_queue, INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH);
    EB_DELETE_PTR_ARRAY(obj->hl_rate_control_historgram_queue, HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH);
    EB_DELETE_PTR_ARRAY(obj->packetization_reorder_queue, PACKETIZATION_REORDER_QUEUE_MAX_DEPTH);
    EB_FREE_ARRAY(obj->rate_control_tables_array);
}

EbErrorType encode_context_ctor(
    EncodeContext *encode_context_ptr,
    EbPtr object_init_data_ptr)
{
    uint32_t pictureIndex;
    EbErrorType return_error = EB_ErrorNone;

    encode_context_ptr->dctor = encode_context_dctor;

    object_init_data_ptr = 0;
    CHECK_REPORT_ERROR(
        (object_init_data_ptr == 0),
        encode_context_ptr->app_callback_ptr,
        EB_ENC_EC_ERROR29);

    EB_CREATE_MUTEX(encode_context_ptr->total_number_of_recon_frame_mutex);
    EB_ALLOC_PTR_ARRAY(encode_context_ptr->picture_decision_reorder_queue, PICTURE_DECISION_REORDER_QUEUE_MAX_DEPTH);

    for (pictureIndex = 0; pictureIndex < PICTURE_DECISION_REORDER_QUEUE_MAX_DEPTH; ++pictureIndex) {
        EB_NEW(encode_context_ptr->picture_decision_reorder_queue[pictureIndex],
            picture_decision_reorder_entry_ctor,
            pictureIndex);
    }

    EB_ALLOC_PTR_ARRAY(encode_context_ptr->picture_manager_reorder_queue,  PICTURE_MANAGER_REORDER_QUEUE_MAX_DEPTH);

    for (pictureIndex = 0; pictureIndex < PICTURE_MANAGER_REORDER_QUEUE_MAX_DEPTH; ++pictureIndex) {
        EB_NEW(encode_context_ptr->picture_manager_reorder_queue[pictureIndex], picture_manager_reorder_entry_ctor, pictureIndex);
    }

    EB_ALLOC_PTR_ARRAY(encode_context_ptr->pre_assignment_buffer, PRE_ASSIGNMENT_MAX_DEPTH);

    EB_ALLOC_PTR_ARRAY(encode_context_ptr->input_picture_queue, INPUT_QUEUE_MAX_DEPTH);

    for (pictureIndex = 0; pictureIndex < INPUT_QUEUE_MAX_DEPTH; ++pictureIndex) {
         EB_NEW(encode_context_ptr->input_picture_queue[pictureIndex], input_queue_entry_ctor);
    }

    EB_ALLOC_PTR_ARRAY(encode_context_ptr->reference_picture_queue, REFERENCE_QUEUE_MAX_DEPTH);

    for (pictureIndex = 0; pictureIndex < REFERENCE_QUEUE_MAX_DEPTH; ++pictureIndex) {
        EB_NEW(encode_context_ptr->reference_picture_queue[pictureIndex], reference_queue_entry_ctor);
    }

    EB_ALLOC_PTR_ARRAY(encode_context_ptr->picture_decision_pa_reference_queue, PICTURE_DECISION_PA_REFERENCE_QUEUE_MAX_DEPTH);

    for (pictureIndex = 0; pictureIndex < PICTURE_DECISION_PA_REFERENCE_QUEUE_MAX_DEPTH; ++pictureIndex) {
        EB_NEW(encode_context_ptr->picture_decision_pa_reference_queue[pictureIndex], pa_reference_queue_entry_ctor);
    }

    EB_ALLOC_PTR_ARRAY(encode_context_ptr->initial_rate_control_reorder_queue, INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH);

    for (pictureIndex = 0; pictureIndex < INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH; ++pictureIndex) {
        EB_NEW(encode_context_ptr->initial_rate_control_reorder_queue[pictureIndex],
            initial_rate_control_reorder_entry_ctor,
            pictureIndex);
    }

    EB_ALLOC_PTR_ARRAY(encode_context_ptr->hl_rate_control_historgram_queue, HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH);

    for (pictureIndex = 0; pictureIndex < HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH; ++pictureIndex) {
        EB_NEW(encode_context_ptr->hl_rate_control_historgram_queue[pictureIndex],
            hl_rate_control_histogram_entry_ctor,
            pictureIndex);
    }
    // HLRateControl Historgram Queue Mutex
    EB_CREATE_MUTEX(encode_context_ptr->hl_rate_control_historgram_queue_mutex);

    EB_ALLOC_PTR_ARRAY(encode_context_ptr->packetization_reorder_queue, PACKETIZATION_REORDER_QUEUE_MAX_DEPTH);

    for (pictureIndex = 0; pictureIndex < PACKETIZATION_REORDER_QUEUE_MAX_DEPTH; ++pictureIndex) {
        EB_NEW(encode_context_ptr->packetization_reorder_queue[pictureIndex],
            packetization_reorder_entry_ctor,
            pictureIndex);
    }

    encode_context_ptr->current_input_poc = -1;
    encode_context_ptr->initial_picture = EB_TRUE;

    // Sequence Termination Flags
    encode_context_ptr->terminating_picture_number = ~0u;

    // Signalling the need for a td structure to be written in the bitstream - on when the sequence starts
    encode_context_ptr->td_needed = EB_TRUE;

    // Rate Control Bit Tables
    EB_MALLOC_ARRAY(encode_context_ptr->rate_control_tables_array, TOTAL_NUMBER_OF_INITIAL_RC_TABLES_ENTRY);

    return_error = rate_control_tables_init(encode_context_ptr->rate_control_tables_array);
    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    // RC Rate Table Update Mutex
    EB_CREATE_MUTEX(encode_context_ptr->rate_table_update_mutex);

    EB_CREATE_MUTEX(encode_context_ptr->sc_buffer_mutex);
    encode_context_ptr->enc_mode                      = SPEED_CONTROL_INIT_MOD;
    encode_context_ptr->previous_selected_ref_qp      = 32;
    encode_context_ptr->max_coded_poc_selected_ref_qp = 32;

    EB_CREATE_MUTEX(encode_context_ptr->shared_reference_mutex);
    return EB_ErrorNone;
}
