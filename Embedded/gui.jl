
using Dash
using DashHtmlComponents
using DashCoreComponents
using Plots

function powplot(n)
		
    p = plot(x -> x^n, label = "y=x^$n", xlims=[0,1])
    figure = (data = Plots.plotly_series(p), layout = Plots.plotly_layout(p))
    figure
end

app =
    dash(external_stylesheets = ["https://codepen.io/chriddyp/pen/bWLwgP.css"])

app.layout = html_div(style = Dict(:width => "50%")) do
    html_h1("Sonar Imaging system"),
    html_button(id = "submit-button-state", children = "submit", n_clicks = 0),
    html_div(id = "output-state"),
    html_br(),
    dcc_graph(id = "power", figure = powplot(1))
end

callback!(
    app,
    Output("power", "figure"),
    Input("submit-button-state", "n_clicks"),
    ) do n_clicks
    	powplot(n_clicks)
    end

run_server(app)
