#include "FormManuelStyring.h"
#include "ui_FormManuelStyring.h"

FormManuelStyring::FormManuelStyring(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FormManuelStyring)
{
    ui->setupUi(this);
}

FormManuelStyring::~FormManuelStyring()
{
    delete ui;
}


