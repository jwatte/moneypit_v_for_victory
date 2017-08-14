#if !defined(widget_h)
#define widget_h

#include <string>

class Widget {
    public:
        Widget(float x, float y, char const *label = "", void (*fn)(Widget *) = NULL, float w = 0.15f, float h = 0.075f);
        float x_;
        float y_;
        float w_;
        float h_;
        std::string label_;
        bool hilite_;
        bool lit_;
        bool drawBackground_;
        bool clickable_;
        bool warned_;
        void (*clickedFn_)(Widget *);

        virtual void draw();
        virtual bool test(float fx, float fy);
        virtual void movement(bool inside, float fx, float fy);
        virtual void click(bool down);
        virtual void clicked();
        virtual void drawBack();
        virtual void drawFront();
        virtual void backColor();
};

class TextWidget : public Widget {
    public:
        TextWidget(float fx, float fy, char const *label, float fw = 0.3f, float fh = 0.075f);
};

class ColorPickerWidget : public Widget {
    public:
        ColorPickerWidget(float left, float right, float top, float bottom);
        void draw() override;
        void movement(bool inside, float fx, float fy) override;
};

void drawText(char const *text, float x, float y, float r = 0, float g = 0, float b = 0, float a = 1);

#endif

