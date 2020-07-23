var search_dict = {};
var removed_list = [];

// Extended Search so you can search with <, <= , >, >=
$.fn.dataTable.ext.search.push(
    function (settings, data, dataIndex) {

        var show_draw_row = true;

        if (removed_list.indexOf(data[0]) >= 0) {
            return false;
        }

        for (var key in search_dict) {
            if (search_dict.hasOwnProperty(key)) {
                var val = search_dict[key];
                var data_el = data[key];

                if (data_el === "-") {
                    show_draw_row = false;
                }
                else if (val.startsWith(">=")) {
                    var numb = parseFloat(val.substring(2));
                    if (data_el < numb) {
                        show_draw_row = false;
                    }
                }
                else if (val.startsWith(">")) {
                    var numb = parseFloat(val.substring(1));
                    if (data_el <= numb) {
                        show_draw_row = false;
                    }
                }
                else if (val.startsWith("<=")) {
                    var numb = parseFloat(val.substring(2));
                    if (data_el > numb) {
                        show_draw_row = false;
                    }
                }
                else if (val.startsWith("<")) {
                    var numb = parseFloat(val.substring(1));
                    if (data_el >= numb) {
                        show_draw_row = false;
                    }
                }
            }
        }
        return show_draw_row;
    }
);


$(document).ready(function() {

    // ###################################################
    // RANDOM MOTIVATIONAL GIF
    // ###################################################
	const giphy = {
		baseURL: "https://api.giphy.com/v1/gifs/",
		key: "QvapRoh7tL9BtjBNMND7tUnlr3WCXg26",
		tag: "teamwork",
		type: "random",
		rating: "pg-13"
	};

	const $gif_wrap = $("#gif-wrap");

	let giphyURL = encodeURI(
		giphy.baseURL +
			giphy.type +
			"?api_key=" +
			giphy.key +
			"&tag=" +
			giphy.tag +
			"&rating=" +
			giphy.rating
	);

	var newGif = () => $.getJSON(giphyURL, json => renderGif(json.data));
	var renderGif = _giphy => {
		$gif_wrap.css({
			"background-image": 'url("' + _giphy.image_original_url + '")'
		});
	};

	newGif();

    // ###################################################
    // DATATABLE
    // ###################################################
    $('#overviewTable tfoot th').each(function () {
        var title = $(this).text();
        $(this).html('<input type="text" class="form-control form-control-sm" placeholder="Filter ' + title + '" />');
    });

    var table = $('#overviewTable').DataTable({
        scrollY: '70vh',
        scrollCollapse: true,
        paging: false,
        scrollX: true,
        info: "",
        dom: 'l<"toolbar">frtip',
        columnDefs: [
            { targets: [0, 1, 2, 3, 4], visible: true },
            { targets: "_all", visible: false }
        ]
    });

    table.columns().every(function () {
        var that = this;
        $('input', this.footer()).on('keyup change', function () {

            var cidx = that[0];
            var pre_str = this.value.replace(/\s/g, "");

            if (pre_str.startsWith(">") || pre_str.startsWith("<")) {
                search_dict[cidx] = pre_str;
                table.draw();

            }
            else {
                if (!pre_str || 0 === pre_str.length) {
                    delete search_dict[cidx];
                    table.draw();
                }
                if (that.search() !== this.value) {
                    that
                        .search(this.value)
                        .draw();
                }
            }
        });
    });

    // ###################################################
    // TOGGLE BUTTONS
    // ###################################################
    function toggle(button, redraw=true) {
        // Get the column API object
        var column = table.column(button.attr('data-column'));
        // Toggle the visibility
        column.visible(!column.visible(), redraw);

        if (column.visible()) {
            button.removeClass("btn-outline-dark");
            button.addClass("btn-dark");
        } else {
            button.removeClass("btn-dark");
            button.addClass("btn-outline-dark");
        }
    };

    $('a.toggle-button').on('click', function (e) {
        e.preventDefault();
        toggle($(this));
    });

    $('a.toggle-label').on('click', function (e) {
        e.preventDefault();
        var label = $(this).attr('data-label');
        var buttons = $('a[data-label-parent="' + label + '"]');
        var n = buttons.length;
        buttons.each(function (i, obj) {
            if (i != n-1) {
                toggle($(obj), false);
            } else {
                toggle($(obj));
            }
        });
    });

    $('a.toggle-metric').on('click', function (e) {
        e.preventDefault();
        var metric = $(this).attr('data-metric');
        var buttons = $('a[data-metric-parent="' + metric + '"]');
        var n = buttons.length;
        buttons.each(function (i, obj) {
            if (i != n-1) {
                toggle($(obj), false);
            } else {
                toggle($(obj));
            }
        });
    });
});
