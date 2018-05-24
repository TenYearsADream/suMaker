/*
    A Simple variable setting modal dialogs modifed by messagedialog
	This dialog is written by Yuan Yao<fly2mars@gmail.com>

    NanoGUI was developed by Wenzel Jakob <wenzel@inf.ethz.ch>.
    The widget drawing code is based on the NanoVG demo application
    by Mikko Mononen.

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#pragma once

#include <nanogui/window.h>
#include <nanogui/textbox.h>
#include <nanogui/checkbox.h>

NAMESPACE_BEGIN(nanogui)

class AddLoadDialog : public Window {
public:
    enum class Type {
        Information,
        Question,
        Warning
    };

	AddLoadDialog(Widget *parent, Type type, 
		          const std::string & unit="",
		          const std::string &title = "Untitled",
                  const std::string &message = "Message",
                  const std::string &buttonText = "OK",
                  const std::string &altButtonText = "Cancel", 
		          bool altButton = false);

    Label *messageLabel() { return mMessageLabel; }
    const Label *messageLabel() const { return mMessageLabel; }
	
    std::function<void(float*)> callback() const { return mCallback; }
    void setCallback(const std::function<void(float*)> &callback) { mCallback = callback; }
protected:
    std::function<void(float*)> mCallback;
    Label *mMessageLabel;
	ref<TextBox> mTextBox;
	ref<CheckBox> mCheckBox[3];
	enum{V_NUM=4};
	float v[V_NUM];
};

NAMESPACE_END(nanogui)
