// #include "savepathdialog.h"
// #include <QVBoxLayout>
// #include <QGroupBox>
// #include <QDialogButtonBox>

// SavePathDialog::SavePathDialog(QWidget *parent) : QDialog(parent) {
//     setWindowTitle("Save Path Options");

//     auto *layout = new QVBoxLayout(this);

//     // ---- Format ----
//     QGroupBox *formatBox = new QGroupBox("Format");
//     QVBoxLayout *formatLayout = new QVBoxLayout(formatBox);
//     xmlButton = new QRadioButton("XML");
//     csvButton = new QRadioButton("CSV");
//     xmlButton->setChecked(true);
//     formatLayout->addWidget(xmlButton);
//     formatLayout->addWidget(csvButton);

//     // ---- Content ----
//     QGroupBox *contentBox = new QGroupBox("Content");
//     QVBoxLayout *contentLayout = new QVBoxLayout(contentBox);
//     allJointsButton = new QRadioButton("All joints");
//     controlledJointsButton = new QRadioButton("Only controlled joints");
//     allJointsButton->setChecked(true);
//     contentLayout->addWidget(allJointsButton);
//     contentLayout->addWidget(controlledJointsButton);

//     // ---- Buttons ----
//     QDialogButtonBox *buttons =
//         new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

//     connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
//     connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

//     layout->addWidget(formatBox);
//     layout->addWidget(contentBox);
//     layout->addWidget(buttons);
// }

// SavePathDialog::Format SavePathDialog::getFormat() const {
//     return xmlButton->isChecked() ? XML : CSV;
// }

// bool SavePathDialog::onlyControlledJoints() const {
//     return controlledJointsButton->isChecked();
// }

#include "savepathdialog.h"
#include <QVBoxLayout>
#include <QDialogButtonBox>

SavePathDialog::SavePathDialog(QWidget *parent) : QDialog(parent) {
    setWindowTitle("Save Path Options");

    auto *layout = new QVBoxLayout(this);

    // ---- Format ----
    QGroupBox *formatBox = new QGroupBox("Format");
    QVBoxLayout *formatLayout = new QVBoxLayout(formatBox);
    xmlButton = new QRadioButton("XML");
    csvButton = new QRadioButton("CSV");
    xmlButton->setChecked(true);
    formatLayout->addWidget(xmlButton);
    formatLayout->addWidget(csvButton);

    connect(xmlButton, &QRadioButton::toggled, this, &SavePathDialog::updateContentVisibility);
    connect(csvButton, &QRadioButton::toggled, this, &SavePathDialog::updateContentVisibility);

    layout->addWidget(formatBox);

    // ---- Content ----
    contentBox = new QGroupBox("Content");
    QVBoxLayout *contentLayout = new QVBoxLayout(contentBox);
    allJointsButton = new QRadioButton("All joints");
    controlledJointsButton = new QRadioButton("Only controlled joints");
    allJointsButton->setChecked(true);
    contentLayout->addWidget(allJointsButton);
    contentLayout->addWidget(controlledJointsButton);

    layout->addWidget(contentBox);

    // ---- Buttons ----
    QDialogButtonBox *buttons =
        new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    layout->addWidget(buttons);

    // Inicialitza visibilitat segons XML/CSV
    updateContentVisibility();
}

SavePathDialog::Format SavePathDialog::getFormat() const {
    return xmlButton->isChecked() ? XML : CSV;
}

bool SavePathDialog::onlyControlledJoints() const {
    return controlledJointsButton->isChecked();
}

void SavePathDialog::updateContentVisibility() {
    // Només mostrar el grup "Content" si CSV està seleccionat
    bool isCSV = csvButton->isChecked();
    contentBox->setVisible(isCSV);

    // Si no és CSV, assegurar-se que "All joints" està seleccionat
    if (!isCSV) {
        allJointsButton->setChecked(true);
    }
}
