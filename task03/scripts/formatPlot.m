function [] = formatPlot(fig,font,fontsize)

ax = findall(allchild(fig),'Type','Axes');
textarrows = findall(allchild(fig),'Type','TextArrow');

set(ax,'XGrid','on');
set(ax,'YGrid','on');

set(ax,'FontName',font);
set(ax,'FontSize',fontsize);

set(textarrows,'FontName',font);
set(textarrows,'FontSize',fontsize);