#ifndef FORMMANUELSTYRING_H
#define FORMMANUELSTYRING_H

#include <QWidget>

namespace Ui {
class FormManuelStyring;
}

class FormManuelStyring : public QWidget
{
    Q_OBJECT

public:
    explicit FormManuelStyring(QWidget *parent = nullptr);
    ~FormManuelStyring();

private:
    Ui::FormManuelStyring *ui;
};

#endif // FORMMANUELSTYRING_H
