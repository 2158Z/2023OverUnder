#include "main.h"
#include "pid.h"
namespace pid{
    lv_obj_t * Pspinbox;

    lv_res_t lv_spinbox_increment_event_cb(lv_obj_t *btn){
        lv_spinbox_increment(Pspinbox);
        return LV_RES_OK;
    }

    lv_res_t lv_spinbox_decrement_event_cb(lv_obj_t *btn){
        lv_spinbox_decrement(Pspinbox);
        return LV_RES_OK;
    }

    void initPID(){
        lv_obj_t *miscTab = lv_tabview_add_tab(selector::tabView, "Misc");
        Pspinbox = lv_spinbox_create(miscTab, NULL);

        lv_obj_t * Plabel = lv_label_create(Pspinbox, NULL);
        lv_label_set_text(Plabel, "P");
        lv_obj_align(Plabel, NULL, LV_ALIGN_IN_TOP_LEFT, 50, 0);
        lv_obj_set_width(Pspinbox, 100);
        lv_obj_align(Pspinbox, NULL, LV_ALIGN_IN_TOP_LEFT, 5, 0);
        lv_obj_set_pos(Pspinbox, 55, 100);
        lv_spinbox_set_range(Pspinbox, -1000, 25000);
        lv_spinbox_set_digit_format(Pspinbox, 2, 1);
        lv_spinbox_set_step(Pspinbox, 1);

        lv_coord_t h = lv_obj_get_height(Pspinbox);

        lv_obj_t * btnPlus = lv_btn_create(miscTab, NULL);
        lv_obj_t * labelPlus = lv_label_create(btnPlus, NULL);

        lv_obj_set_size(btnPlus, h/1.5, h/1.5);
        lv_obj_align(btnPlus, Pspinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
        lv_btn_set_action(btnPlus, LV_BTN_ACTION_CLICK, *lv_spinbox_increment_event_cb);
        lv_label_set_text(labelPlus, SYMBOL_PLUS);

        lv_obj_t * btnMinus = lv_btn_create(miscTab, NULL);
        lv_obj_t * labelMinus = lv_label_create(btnMinus, NULL);

        lv_obj_set_size(btnMinus, h/1.5, h/1.5);
        lv_obj_align(btnMinus, Pspinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
        lv_btn_set_action(btnMinus, LV_BTN_ACTION_CLICK, *lv_spinbox_decrement_event_cb);
        lv_label_set_text(labelMinus, SYMBOL_MINUS);

        // lv_obj_t * Ispinbox = lv_spinbox_create(miscTab, Pspinbox);
        // lv_obj_t * Ilabel = lv_label_create(Ispinbox, Plabel);
        // lv_label_set_text(Plabel, "I");
        // lv_obj_align(Ispinbox, NULL, LV_ALIGN_IN_TOP_LEFT, 205, 10);
    }      
}