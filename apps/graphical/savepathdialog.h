#pragma once
#include <QDialog>
#include <QRadioButton>
#include <QPushButton>
#include <QGroupBox>

class SavePathDialog : public QDialog {
    Q_OBJECT
public:
    enum Format { XML, CSV };

    SavePathDialog(QWidget *parent = nullptr);

    Format getFormat() const;
    bool onlyControlledJoints() const;

private:
    QRadioButton *xmlButton;
    QRadioButton *csvButton;
    QGroupBox *contentBox;
    QRadioButton *allJointsButton;
    QRadioButton *controlledJointsButton;

    void updateContentVisibility();
};
