/*
    src/messagedialog.cpp -- Simple "OK" or "Yes/No"-style modal dialogs

    NanoGUI was developed by Wenzel Jakob <wenzel@inf.ethz.ch>.
    The widget drawing code is based on the NanoVG demo application
    by Mikko Mononen.

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include "addLoadDlg.h"
#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/entypo.h>
#include <nanogui/label.h>

#include <iostream>

NAMESPACE_BEGIN(nanogui)
AddLoadDialog::AddLoadDialog(Widget * parent, Type type,
	          const std::string & unit,
	          const std::string & title, 
	          const std::string & message, 
	          const std::string & buttonText, 
	          const std::string & altButtonText, 
	          bool altButton) : Window(parent, title)
{
	//init value
	for (int i = 0; i < V_NUM; i++) {
		v[i] = 0;
	}
	//init UI
	setLayout(new BoxLayout(Orientation::Vertical,
		Alignment::Middle, 10, 10));
	setModal(true);
	
	Widget *panel1 = new Widget(this);
	panel1->setLayout(new BoxLayout(Orientation::Horizontal,
		Alignment::Middle, 10, 15));
	int icon = 0;
	switch (type) {
	case Type::Information: icon = ENTYPO_ICON_CIRCLED_INFO; break;
	case Type::Question: icon = ENTYPO_ICON_CIRCLED_HELP; break;
	case Type::Warning: icon = ENTYPO_ICON_WARNING; break;
	}
	Label *iconLabel = new Label(panel1, std::string(utf8(icon).data()), "icons");
	iconLabel->setFontSize(50);
	mMessageLabel = new Label(panel1, message);
	mMessageLabel->setFixedWidth(200);

	//test
	Widget *panel3 = new Widget(this);
	GridLayout *layout =
		new GridLayout(Orientation::Horizontal, 2,
			Alignment::Middle, 15, 5);
	layout->setColAlignment(
	{ Alignment::Maximum, Alignment::Fill });
	layout->setSpacing(0, 10);
	panel3->setLayout(layout);
	/* FP widget */ 
	{
		//variable Edit
		if (!unit.empty()) {
			new Label(panel3, "value :", "sans-bold");
			mTextBox = new  TextBox(panel3);
			mTextBox->setEditable(true);
			mTextBox->setFixedSize(Vector2i(100, 20));
			mTextBox->setValue("50");
			mTextBox->setUnits(unit);
			mTextBox->setDefaultValue("0.0");
			mTextBox->setFontSize(16);
			mTextBox->setFormat("[-]?[0-9]*\\.?[0-9]+");
			new Label(panel3, "direction :", "sans-bold");
			Widget *panelCheck = new Widget(panel3);
			layout = new GridLayout(Orientation::Horizontal, 6,
					Alignment::Middle, 5, 5);
			layout->setColAlignment(
			{ Alignment::Maximum, Alignment::Fill });
			layout->setSpacing(0, 5);
			panelCheck->setLayout(layout);
			std::string axis[] = { "x", "y", "z" };
			for (int i = 0; i < 3; i++) {
				mCheckBox[i] = new CheckBox(panelCheck, axis[i]);
				mCheckBox[i]->setFontSize(18);
				mCheckBox[i]->setChecked(false);
				
			}			
		}
		
	}
	//test
	
	Widget *panel2 = new Widget(this);
	panel2->setLayout(new BoxLayout(Orientation::Horizontal,
		Alignment::Middle, 0, 15));

	if (altButton) {
		Button *button = new Button(panel2, altButtonText, ENTYPO_ICON_CIRCLED_CROSS);
		button->setCallback([&] { dispose(); });
	}
	Button *button = new Button(panel2, buttonText, ENTYPO_ICON_CHECK);

	mCallback = nullptr;

	button->setCallback([&] {
		if (mCallback) {
			v[0] = std::stof(mTextBox->value());
			int nChecked = 0;
			for (int i = 0; i < 3; i++) {
				if (mCheckBox[i]->checked()) {
					nChecked++;
					v[i + 1] = 1;
				}
			}
			if (nChecked > 1) {
				mMessageLabel->setCaption("Error: Only one direction can be choosed!");
				return;
			}
			else if (nChecked < 1) {
				mMessageLabel->setCaption("Error: One direction must be choosed!");
				return;
			}
			mCallback(v);
		}
		dispose();
	});
	center();
	requestFocus();
}


NAMESPACE_END(nanogui)

